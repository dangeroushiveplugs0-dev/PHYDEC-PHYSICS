#include "phydec/secondary_motion.hpp"
#include "phydec/experimental/exact_spring.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using phydec::SecondaryMotionChain;
using phydec::SecondaryMotionSettings;
using phydec::Vec3;
using phydec::experimental::ExactSpringSettings;
using phydec::experimental::ExactSpringState;
using phydec::experimental::step_exact_spring;

namespace {
constexpr float pi = 3.14159265358979323846F;
void require(bool ok, const char* message) {
    if (!ok) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
Vec3 rotate_z(const Vec3& p, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
}
float distance(const Vec3& a, const Vec3& b) {
    return phydec::length(a - b);
}
}

int main() {
    constexpr float dt = 1.0F / 120.0F;
    constexpr int frames = 1200;
    const std::vector<Vec3> rest{{0, 0, 0}, {0, -0.35F, 0}, {0, -0.7F, 0}, {0, -1.05F, 0}};
    SecondaryMotionChain chain(rest);
    SecondaryMotionSettings settings;
    settings.fixed_delta_seconds = dt;
    settings.gravity = {0.0F, 0.0F, -1.5F};
    settings.constraint_iterations = 8;

    ExactSpringSettings spring;
    spring.frequency_hz = 2.1F;
    spring.damping_ratio = 0.32F;
    spring.gravity = settings.gravity;
    std::vector<ExactSpringState> exact(rest.size() - 1);

    std::vector<Vec3> older_anchors = rest;
    std::vector<Vec3> previous_anchors = rest;
    std::vector<Vec3> anchors = rest;
    float max_chain_length_error = 0.0F;
    float max_exact_chain_length_error = 0.0F;
    float max_exact_offset = 0.0F;

    for (int frame = 1; frame <= frames; ++frame) {
        const float t = static_cast<float>(frame) * dt;
        // A moving root plus an animated rotation. This is a deterministic
        // comparison fixture, not a claim that either solver is universally better.
        const float angle = 0.75F * std::sin(2.0F * pi * 0.8F * t);
        const Vec3 root{0.18F * std::sin(2.0F * pi * 0.55F * t),
                        0.10F * std::cos(2.0F * pi * 0.55F * t), 0.0F};
        for (std::size_t i = 0; i < rest.size(); ++i) {
            anchors[i] = root + rotate_z(rest[i], angle);
        }
        chain.set_anchors(anchors);
        chain.step(settings);

        for (std::size_t i = 1; i < anchors.size(); ++i) {
            const Vec3 parent_acceleration = frame > 1
                ? (anchors[i - 1] - previous_anchors[i - 1] * 2.0F +
                   older_anchors[i - 1]) / (dt * dt)
                : Vec3{};
            // Estimate acceleration from the current and two prior samples.
            // At the first frame use zero forcing rather than a fabricated history.
            step_exact_spring(exact[i - 1], dt, spring, parent_acceleration);
            const Vec3 exact_position = anchors[i] + exact[i - 1].offset;
            const Vec3 previous_exact_position = i == 1
                ? anchors[0] : anchors[i - 1] + exact[i - 2].offset;
            max_exact_chain_length_error = std::max(max_exact_chain_length_error,
                std::abs(distance(exact_position, previous_exact_position) -
                         distance(rest[i], rest[i - 1])));
            max_exact_offset = std::max(max_exact_offset, phydec::length(exact[i - 1].offset));
        }
        const auto& points = chain.points();
        for (std::size_t i = 1; i < points.size(); ++i) {
            max_chain_length_error = std::max(max_chain_length_error,
                std::abs(distance(points[i].position, points[i - 1].position) -
                         distance(rest[i], rest[i - 1])));
        }
        older_anchors = previous_anchors;
        previous_anchors = anchors;
    }

    require(chain.valid(), "constrained chain remains finite during rotating-parent fixture");
    require(std::isfinite(max_exact_offset), "exact oscillator offsets remain finite");
    // The constrained chain's purpose includes preserving segment lengths. Keep
    // a tolerance because constraints are iterative and motion is dynamic.
    require(max_chain_length_error < 0.03F,
            "constrained chain should keep segment lengths near their rest lengths");

    std::cout << "Linked-chain comparison fixture passed over " << frames << " frames.\n"
              << "max constrained-chain segment error: " << max_chain_length_error << "\n"
              << "max independent-oscillator adjacent-length error: "
              << max_exact_chain_length_error << "\n"
              << "max independent-oscillator offset: " << max_exact_offset << "\n"
              << "Interpretation: independent springs and constrained chains have different "
                 "semantics; metrics are not a universal quality ranking.\n";
    return 0;
}
