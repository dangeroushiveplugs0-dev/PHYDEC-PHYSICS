#include "phydec/secondary_motion.hpp"

#include <cassert>
#include <cmath>
#include <vector>

int main() {
    using phydec::PlaneCollider;
    using phydec::SecondaryMotionChain;
    using phydec::SecondaryMotionSettings;
    using phydec::Vec3;

    SecondaryMotionChain chain({{0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, -1.0F}});
    assert(chain.points().size() == 2);
    assert(chain.valid());

    SecondaryMotionSettings settings;
    settings.gravity = {0.0F, 0.0F, -9.81F};
    settings.fixed_delta_seconds = 1.0F / 60.0F;

    for (int i = 0; i < 120; ++i) {
        chain.set_anchors({{0.0F, 0.0F, 0.0F}, {0.1F, 0.0F, -1.0F}});
        chain.step(settings);
        assert(chain.valid());
        assert(std::isfinite(chain.points()[1].position.z));
    }

    // The root remains attached to the animated anchor.
    const auto root = chain.points().front().position;
    assert(std::abs(root.x) < 1.0e-5F);
    assert(std::abs(root.y) < 1.0e-5F);
    assert(std::abs(root.z) < 1.0e-5F);

    // Invalid delta time is safely ignored.
    const auto before = chain.points()[1].position;
    settings.fixed_delta_seconds = 0.0F;
    chain.step(settings);
    const auto after = chain.points()[1].position;
    assert(std::abs(before.x - after.x) < 1.0e-6F);
    assert(std::abs(before.y - after.y) < 1.0e-6F);
    assert(std::abs(before.z - after.z) < 1.0e-6F);

    // Contacts are applied inside the motion solver, not just exposed as helpers.
    SecondaryMotionChain colliding_chain({{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}});
    SecondaryMotionSettings collision_settings;
    collision_settings.gravity = {};
    collision_settings.fixed_delta_seconds = 1.0F / 60.0F;
    collision_settings.plane_colliders.push_back(PlaneCollider{{}, {0.0F, 0.0F, 1.0F}});
    colliding_chain.step(collision_settings);
    assert(colliding_chain.valid());
    assert(colliding_chain.points()[1].position.z >=
           colliding_chain.points()[1].collision_radius - 1.0e-4F);
    assert(std::abs(colliding_chain.points()[0].position.z - 1.0F) < 1.0e-5F);
    return 0;
}
