#pragma once

#include <algorithm>
#include <cmath>
#include "phydec/math/vec3.hpp"

namespace phydec {

// Initial analytic collider. Mesh/proxy colliders will build on this query contract.
struct SphereCollider {
    Vec3 center{};
    float radius{0.5F};
};

struct Contact {
    Vec3 position{};
    Vec3 normal{};
    float penetration_depth{0.0F};
    bool touching{false};
};

// Pushes a point outside a sphere expanded by particle_radius.
// Returns a contact only when overlap occurs; invalid radii are treated as no contact.
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

    // A stable fallback normal avoids division by zero at the sphere center.
    contact.normal = distance > 1.0e-7F ? offset / distance : Vec3{0.0F, 1.0F, 0.0F};
    contact.penetration_depth = combined_radius - distance;
    point += contact.normal * contact.penetration_depth;
    contact.position = sphere.center + contact.normal * sphere.radius;
    contact.touching = true;
    return contact;
}

} // namespace phydec
