#include "phydec/experimental/exact_spring.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using phydec::Vec3;
using phydec::experimental::ExactSpringSettings;
using phydec::experimental::ExactSpringState;
using phydec::experimental::step_exact_spring;

namespace {
bool near(float a, float b, float tolerance) {
    return std::abs(a - b) <= tolerance;
}
bool near_vec(const Vec3& a, const Vec3& b, float tolerance) {
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
           near(a.z, b.z, tolerance);
}
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
}

int main() {
    // Exact constant-coefficient integration should be effectively independent
    // of how a fixed total interval is partitioned into steps.
    for (const float damping : {0.0F, 0.14F, 1.0F, 1.7F, 8.0F}) {
        ExactSpringSettings settings;
        settings.frequency_hz = 2.2F;
        settings.damping_ratio = damping;
        settings.gravity = {0.0F, -9.81F, 1.25F};
        settings.relative_size = 0.8F;
        ExactSpringState single{{0.4F, -0.2F, 0.7F}, {1.1F, 0.3F, -0.6F}};
        ExactSpringState subdivided = single;
        step_exact_spring(single, 1.0F, settings, {0.25F, -0.5F, 0.75F});
        for (int i = 0; i < 100; ++i) {
            step_exact_spring(subdivided, 0.01F, settings, {0.25F, -0.5F, 0.75F});
        }
        require(near_vec(single.offset, subdivided.offset, 2.0e-4F),
                "constant-forcing offset should be frame-partition invariant");
        require(near_vec(single.velocity, subdivided.velocity, 2.0e-4F),
                "constant-forcing velocity should be frame-partition invariant");
        require(is_finite(single.offset) && is_finite(single.velocity),
                "damping regimes must remain finite");
    }

    // Under constant gravity, the equilibrium is gravity / omega^2.
    ExactSpringSettings gravity;
    gravity.frequency_hz = 1.5F;
    gravity.damping_ratio = 0.8F;
    gravity.gravity = {0.0F, -9.81F, 0.0F};
    ExactSpringState sag{};
    for (int i = 0; i < 1800; ++i) step_exact_spring(sag, 1.0F / 60.0F, gravity);
    const float omega = 6.2831853071795864769F * gravity.frequency_hz;
    const float expected_y = gravity.gravity.y / (omega * omega);
    require(near(sag.offset.y, expected_y, 2.0e-4F),
            "gravity sag should converge to the analytic equilibrium");
    require(std::abs(sag.velocity.y) < 2.0e-4F,
            "gravity sag should settle to near-zero velocity");

    // Invalid time and non-finite input are no-ops rather than poisoning state.
    ExactSpringState unchanged{{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}};
    const ExactSpringState original = unchanged;
    step_exact_spring(unchanged, 0.0F, gravity);
    require(near_vec(unchanged.offset, original.offset, 0.0F) &&
            near_vec(unchanged.velocity, original.velocity, 0.0F),
            "zero dt should leave state unchanged");

    std::cout << "Exact spring prototype tests passed.\n";
    return 0;
}
