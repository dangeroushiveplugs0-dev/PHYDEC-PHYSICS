#include "phydec/secondary_motion.hpp"

#include <algorithm>
#include <cmath>

namespace phydec {

SecondaryMotionChain::SecondaryMotionChain(std::vector<Vec3> initial_positions) {
    points_.reserve(initial_positions.size());
    for (std::size_t i = 0; i < initial_positions.size(); ++i) {
        MotionPoint point;
        point.position = initial_positions[i];
        point.previous_position = initial_positions[i];
        point.anchor = initial_positions[i];
        if (i > 0) {
            point.segment_length = length(initial_positions[i] - initial_positions[i - 1]);
        }
        points_.push_back(point);
    }
}

void SecondaryMotionChain::set_anchors(const std::vector<Vec3>& animated_positions) {
    if (animated_positions.size() != points_.size()) return;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        points_[i].anchor = animated_positions[i];
    }
}

void SecondaryMotionChain::step(const SecondaryMotionSettings& settings) {
    const float dt = settings.fixed_delta_seconds;
    if (!(dt > 0.0F) || !std::isfinite(dt) || points_.empty()) return;

    points_[0].position = points_[0].anchor;
    points_[0].previous_position = points_[0].anchor;

    for (std::size_t i = 1; i < points_.size(); ++i) {
        auto& point = points_[i];
        const float safe_mass = std::max(point.mass, 1.0e-4F);
        const Vec3 velocity = (point.position - point.previous_position) *
            (1.0F - std::clamp(point.damping, 0.0F, 1.0F));
        const Vec3 old_position = point.position;
        const Vec3 acceleration = settings.gravity + settings.external_acceleration;
        const Vec3 spring = (point.anchor - point.position) * std::max(point.stiffness, 0.0F);
        point.position += velocity + (acceleration + spring / safe_mass) * (dt * dt);
        point.previous_position = old_position;
    }

    const unsigned iterations = std::max(settings.constraint_iterations, 1U);
    for (unsigned iteration = 0; iteration < iterations; ++iteration) {
        points_[0].position = points_[0].anchor;
        for (std::size_t i = 1; i < points_.size(); ++i) {
            auto& parent = points_[i - 1];
            auto& child = points_[i];
            const Vec3 delta = child.position - parent.position;
            const float distance = length(delta);
            if (distance <= 1.0e-7F) continue;
            const Vec3 correction = delta * ((distance - child.segment_length) / distance);
            if (i == 1) {
                child.position -= correction;
            } else {
                parent.position += correction * 0.5F;
                child.position -= correction * 0.5F;
            }
        }

        // Apply all collision types inside each constraint iteration.
        // Shift previous_position with the positional correction to avoid adding
        // fake velocity solely because a contact projected the point outward.
        for (std::size_t i = 1; i < points_.size(); ++i) {
            auto& point = points_[i];
            const auto preserve_velocity_contact = [&point, &settings](auto&& resolver) {
                const Vec3 before = point.position;
                const Contact contact = resolver();
                if (!contact.touching) return;

                // First remove artificial velocity introduced by positional projection.
                point.previous_position += point.position - before;

                // Verlet displacement represents per-step velocity.
                const Vec3 displacement = point.position - point.previous_position;
                const Vec3 response = apply_contact_response(
                    displacement, contact, settings.collision_friction,
                    settings.collision_restitution);
                point.previous_position = point.position - response;
            };

            for (const auto& collider : settings.sphere_colliders) {
                preserve_velocity_contact([&]() {
                    return resolve_sphere_contact(point.position, point.collision_radius, collider);
                });
            }
            for (const auto& collider : settings.capsule_colliders) {
                preserve_velocity_contact([&]() {
                    return resolve_capsule_contact(point.position, point.collision_radius, collider);
                });
            }
            for (const auto& collider : settings.plane_colliders) {
                preserve_velocity_contact([&]() {
                    return resolve_plane_contact(point.position, point.collision_radius, collider);
                });
            }
            for (const auto& collider : settings.mesh_colliders) {
                preserve_velocity_contact([&]() {
                    const SweptMeshContact swept = sweep_sphere_against_mesh(
                        point.previous_position, point.position, point.collision_radius, collider);
                    Contact surface_contact;
                    if (swept.hit) {
                        point.position = swept.position;
                        surface_contact.position = swept.position - swept.normal * point.collision_radius;
                        surface_contact.normal = swept.normal;
                        surface_contact.touching = true;
                    } else {
                        surface_contact = resolve_mesh_contact(
                            point.position, point.collision_radius, collider);
                    }
                    if (surface_contact.touching) return surface_contact;
                    return resolve_closed_mesh_contact(
                        point.position, point.collision_radius, collider);
                });
            }
        }
    }
    points_[0].position = points_[0].anchor;
}

bool SecondaryMotionChain::valid() const noexcept {
    for (const auto& point : points_) {
        if (!is_finite(point.position) || !is_finite(point.previous_position) ||
            !is_finite(point.anchor) || !std::isfinite(point.segment_length) ||
            !std::isfinite(point.stiffness) || !std::isfinite(point.damping) ||
            !std::isfinite(point.mass) || !std::isfinite(point.collision_radius) ||
            point.segment_length < 0.0F || point.collision_radius < 0.0F) return false;
    }
    return true;
}

} // namespace phydec
