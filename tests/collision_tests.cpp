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
    return 0;
}
