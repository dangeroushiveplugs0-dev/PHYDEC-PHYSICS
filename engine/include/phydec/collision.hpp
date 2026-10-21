#pragma once

#include <algorithm>
#include <cmath>
#include "phydec/math/vec3.hpp"

namespace phydec {

// Analytic colliders are the first layer of the hybrid collision system.
// Detailed mesh contacts and broad-phase acceleration will be added separately.
struct SphereCollider {
    Vec3 center{};
    float radius{0.5F};
};

struct CapsuleCollider {
    Vec3 start{};
    Vec3 end{0.0F, 1.0F, 0.0F};
    float radius{0.25F};
};

// Plane normal points toward the side where particles are allowed to remain.
struct PlaneCollider {
    Vec3 point{};
    Vec3 normal{0.0F, 0.0F, 1.0F};
};

struct Contact {
    Vec3 position{};
    Vec3 normal{};
    float penetration_depth{0.0F};
    bool touching{false};
};

// Pushes a point outside a sphere expanded by particle_radius.
inline Contact resolve_sphere_contact(Vec3& point, float particle_radius,
                                      const SphereCollider& sphere) noexcept {
    Contact contact;
    if (!is_finite(point) || !is_finite(sphere.center) ||
        !std::isfinite(particle_radius) || !std::isfinite(sphere.radius) ||
        particle_radius < 0.0F || sphere.radius < 0.0F) {
        return contact;
    }

    const Vec3 offset = point - sphere.center;
    const float distance = length(offset);
    const float combined_radius = particle_radius + sphere.radius;
    if (distance >= combined_radius) return contact;

    contact.normal = distance > 1.0e-7F ? offset / distance : Vec3{0.0F, 1.0F, 0.0F};
    contact.penetration_depth = combined_radius - distance;
    point += contact.normal * contact.penetration_depth;
    contact.position = sphere.center + contact.normal * sphere.radius;
    contact.touching = true;
    return contact;
}

// Pushes a point outside a capsule (a line segment swept by a sphere), expanded
// by particle_radius. Degenerate segments behave as sphere colliders.
inline Contact resolve_capsule_contact(Vec3& point, float particle_radius,
                                       const CapsuleCollider& capsule) noexcept {
    Contact contact;
    if (!is_finite(point) || !is_finite(capsule.start) || !is_finite(capsule.end) ||
        !std::isfinite(particle_radius) || !std::isfinite(capsule.radius) ||
        particle_radius < 0.0F || capsule.radius < 0.0F) {
        return contact;
    }

    const Vec3 segment = capsule.end - capsule.start;
    const float segment_length_squared = dot(segment, segment);
    float t = 0.0F;
    if (segment_length_squared > 1.0e-12F) {
        t = std::clamp(dot(point - capsule.start, segment) / segment_length_squared,
                       0.0F, 1.0F);
    }

    const Vec3 closest = capsule.start + segment * t;
    const Vec3 offset = point - closest;
    const float distance = length(offset);
    const float combined_radius = particle_radius + capsule.radius;
    if (distance >= combined_radius) return contact;

    if (distance > 1.0e-7F) {
        contact.normal = offset / distance;
    } else if (segment_length_squared > 1.0e-12F) {
        const Vec3 axis = normalized(segment);
        const Vec3 candidate = std::abs(axis.y) < 0.9F
            ? Vec3{0.0F, 1.0F, 0.0F} : Vec3{1.0F, 0.0F, 0.0F};
        contact.normal = normalized(Vec3{
            axis.y * candidate.z - axis.z * candidate.y,
            axis.z * candidate.x - axis.x * candidate.z,
            axis.x * candidate.y - axis.y * candidate.x
        });
    } else {
        contact.normal = Vec3{0.0F, 1.0F, 0.0F};
    }

    contact.penetration_depth = combined_radius - distance;
    point += contact.normal * contact.penetration_depth;
    contact.position = closest + contact.normal * capsule.radius;
    contact.touching = true;
    return contact;
}

// Resolves one-sided plane contact. The plane's normal is normalized internally;
// points behind the plane are pushed to particle_radius on its allowed side.
inline Contact resolve_plane_contact(Vec3& point, float particle_radius,
                                     const PlaneCollider& plane) noexcept {
    Contact contact;
    if (!is_finite(point) || !is_finite(plane.point) || !is_finite(plane.normal) ||
        !std::isfinite(particle_radius) || particle_radius < 0.0F) {
        return contact;
    }

    const float normal_length = length(plane.normal);
    if (!(normal_length > 1.0e-7F) || !std::isfinite(normal_length)) return contact;
    const Vec3 normal = plane.normal / normal_length;
    const float signed_distance = dot(point - plane.point, normal);
    if (signed_distance >= particle_radius) return contact;

    contact.normal = normal;
    contact.penetration_depth = particle_radius - signed_distance;
    point += normal * contact.penetration_depth;
    contact.position = point - normal * particle_radius;
    contact.touching = true;
    return contact;
}

// Applies simple contact response to Verlet displacement. Friction damps tangent
// motion; restitution reverses inward normal motion. Both coefficients are clamped.
inline Vec3 apply_contact_response(const Vec3& displacement, const Contact& contact,
                                   float friction, float restitution) noexcept {
    if (!contact.touching || !is_finite(displacement) || !is_finite(contact.normal)) {
        return displacement;
    }
    const Vec3 normal = normalized(contact.normal);
    if (length(normal) <= 1.0e-7F) return displacement;

    const float normal_displacement = dot(displacement, normal);
    if (normal_displacement >= 0.0F) return displacement;

    const float safe_friction = std::clamp(friction, 0.0F, 1.0F);
    const float safe_restitution = std::clamp(restitution, 0.0F, 1.0F);
    const Vec3 normal_part = normal * normal_displacement;
    const Vec3 tangent_part = displacement - normal_part;
    return tangent_part * (1.0F - safe_friction) -
           normal_part * safe_restitution;
}

} // namespace phydec
