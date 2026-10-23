#include "phydec/experimental/exact_spring.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using phydec::Vec3;
using phydec::experimental::ExactSpringBone;
using phydec::experimental::ExactSpringSettings;

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
}

int main() {
    ExactSpringSettings settings;
    settings.frequency_hz = 2.0F;
    settings.damping_ratio = 0.35F;
    settings.gravity = {0.0F, 0.0F, -1.5F};

    ExactSpringBone bone;
    const Vec3 start{0.0F, 0.0F, 0.0F};
    require(phydec::length(bone.update(start, 1.0F / 60.0F, settings) - start) < 1.0e-6F,
            "first update initializes at the animated anchor");

    Vec3 output{};
    for (int i = 1; i <= 600; ++i) {
        const float t = static_cast<float>(i) / 120.0F;
        const Vec3 anchor{0.25F * std::sin(2.0F * 3.14159265F * 0.7F * t),
                          0.0F, 0.0F};
        output = bone.update(anchor, 1.0F / 120.0F, settings);
        require(phydec::is_finite(output), "animated-anchor output remains finite");
    }
    require(phydec::length(bone.state().offset) > 1.0e-5F,
            "animated anchor motion excites the spring offset");

    const auto before = bone.state();
    const Vec3 invalid_output = bone.update({NAN, 0.0F, 0.0F}, 1.0F / 120.0F, settings);
    require(phydec::length(bone.state().offset - before.offset) == 0.0F &&
            phydec::length(bone.state().velocity - before.velocity) == 0.0F,
            "invalid anchor does not mutate spring state");
    require(phydec::is_finite(invalid_output), "invalid anchor returns last finite output");

    bone.reset({1.0F, 2.0F, 3.0F});
    require(phydec::length(bone.state().offset) < 1.0e-6F &&
            phydec::length(bone.state().velocity) < 1.0e-6F,
            "reset immediately clears offset and velocity");
    const Vec3 reset_output = bone.update({1.0F, 2.0F, 3.0F}, 1.0F / 60.0F, settings);
    require(phydec::is_finite(reset_output),
            "update after reset returns finite output");

    std::cout << "Experimental exact-spring bone wrapper tests passed.\n";
    return 0;
}
