#include "phydec/collision.hpp"

#include <cassert>
#include <cmath>

int main() {
    using namespace phydec;
    const SphereCollider sphere{{0.0F, 0.0F, 0.0F}, 1.0F};

    Vec3 point{0.5F, 0.0F, 0.0F};
    const Contact hit = resolve_sphere_contact(point, 0.25F, sphere);
    assert(hit.touching);
    assert(std::abs(hit.penetration_depth - 0.75F) < 1.0e-5F);
    assert(std::abs(length(point) - 1.25F) < 1.0e-5F);
    assert(is_finite(hit.normal));

    Vec3 outside{2.0F, 0.0F, 0.0F};
    const Contact miss = resolve_sphere_contact(outside, 0.25F, sphere);
    assert(!miss.touching);
    assert(std::abs(outside.x - 2.0F) < 1.0e-6F);

    Vec3 center{};
    const Contact center_hit = resolve_sphere_contact(center, 0.25F, sphere);
    assert(center_hit.touching);
    assert(is_finite(center));
    assert(std::abs(length(center) - 1.25F) < 1.0e-5F);

    Vec3 invalid_point{};
    const Contact invalid = resolve_sphere_contact(invalid_point, -1.0F, sphere);
    assert(!invalid.touching);

    const CapsuleCollider capsule{{0.0F, -1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, 0.5F};
    Vec3 capsule_point{0.25F, 0.0F, 0.0F};
    const Contact capsule_hit = resolve_capsule_contact(capsule_point, 0.25F, capsule);
    assert(capsule_hit.touching);
    assert(std::abs(capsule_hit.penetration_depth - 0.5F) < 1.0e-5F);
    assert(std::abs(length(capsule_point) - 0.5F) < 1.0e-5F);
    assert(is_finite(capsule_hit.normal));

    Vec3 capsule_miss{2.0F, 0.0F, 0.0F};
    assert(!resolve_capsule_contact(capsule_miss, 0.25F, capsule).touching);

    const CapsuleCollider degenerate{{1.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, 0.5F};
    Vec3 degenerate_point{1.0F, 0.0F, 0.0F};
    const Contact degenerate_hit = resolve_capsule_contact(degenerate_point, 0.25F, degenerate);
    assert(degenerate_hit.touching);
    assert(is_finite(degenerate_point));
    assert(std::abs(length(degenerate_point - degenerate.start) - 0.75F) < 1.0e-5F);

    Vec3 invalid_capsule_point{};
    CapsuleCollider invalid_capsule = capsule;
    invalid_capsule.radius = -1.0F;
    assert(!resolve_capsule_contact(invalid_capsule_point, 0.1F, invalid_capsule).touching);
    return 0;
}
