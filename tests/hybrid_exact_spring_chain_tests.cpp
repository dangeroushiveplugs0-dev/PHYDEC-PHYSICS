#include "phydec/experimental/hybrid_exact_spring_chain.hpp"
#include "phydec/secondary_motion.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

using phydec::Vec3;
using phydec::SecondaryMotionChain;
using phydec::SecondaryMotionSettings;
using phydec::experimental::ExactSpringSettings;
using phydec::experimental::HybridExactSpringChain;

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
float distance(const Vec3& a, const Vec3& b) {
    return phydec::length(a - b);
}
Vec3 rotate_z(const Vec3& p, float angle) {
    const float c = std::cos(angle), s = std::sin(angle);
    return {c * p.x - s * p.y, s * p.x + c * p.y, p.z};
}
}

int main() {
    constexpr float dt = 1.0F / 120.0F;
    constexpr float pi = 3.14159265358979323846F;
    const std::vector<Vec3> rest{{0, 0, 0}, {0, -0.35F, 0}, {0, -0.70F, 0}, {0, -1.05F, 0}};
    HybridExactSpringChain hybrid(rest);
    SecondaryMotionChain production(rest);
    SecondaryMotionSettings production_settings;
    production_settings.fixed_delta_seconds = dt;
    production_settings.gravity = {0.0F, 0.0F, -1.5F};
    production_settings.constraint_iterations = 10;
    ExactSpringSettings settings;
    settings.frequency_hz = 2.1F;
    settings.damping_ratio = 0.32F;
    settings.gravity = {0.0F, 0.0F, -1.5F};

    float max_length_error = 0.0F;
    float max_offset = 0.0F;
    float production_max_length_error = 0.0F;
    float production_max_offset = 0.0F;
    for (int frame = 1; frame <= 1200; ++frame) {
        const float t = static_cast<float>(frame) * dt;
        const float angle = 0.75F * std::sin(2.0F * pi * 0.8F * t);
        const Vec3 root{0.18F * std::sin(2.0F * pi * 0.55F * t),
                        0.10F * std::cos(2.0F * pi * 0.55F * t), 0.0F};
        std::vector<Vec3> anchors;
        for (const Vec3& point : rest) anchors.push_back(root + rotate_z(point, angle));
        require(hybrid.step(anchors, dt, settings, 10), "valid animated pose is accepted");
        production.set_anchors(anchors);
        production.step(production_settings);
        require(production.valid(), "production chain remains finite on the same animated pose");
        const auto& positions = hybrid.positions();
        const auto& production_points = production.points();
        require(positions.size() == rest.size(), "output chain keeps the anchor count");
        for (std::size_t i = 0; i < positions.size(); ++i) {
            require(phydec::is_finite(positions[i]), "all hybrid positions remain finite");
            max_offset = std::max(max_offset, distance(positions[i], anchors[i]));
            production_max_offset = std::max(production_max_offset,
                distance(production_points[i].position, anchors[i]));
            if (i > 0) {
                const float error = std::abs(distance(positions[i], positions[i - 1]) -
                                             distance(rest[i], rest[i - 1]));
                max_length_error = std::max(max_length_error, error);
                const float production_error = std::abs(
                    distance(production_points[i].position, production_points[i - 1].position) -
                    distance(rest[i], rest[i - 1]));
                production_max_length_error = std::max(production_max_length_error,
                                                       production_error);
            }
        }
        require(distance(positions[0], anchors[0]) < 1.0e-6F,
                "animated root remains pinned");
    }
    require(max_length_error < 0.015F,
            "hybrid constraints keep segment lengths near rest length");
    require(max_offset > 1.0e-5F, "hybrid retains a measurable secondary-motion response");
    require(production_max_length_error < 0.03F,
            "production baseline keeps segment lengths near rest lengths");
    require(production_max_offset > 1.0e-5F,
            "production baseline retains measurable secondary motion");

    const auto before = hybrid.positions();
    auto invalid = rest;
    invalid[1].x = std::numeric_limits<float>::quiet_NaN();
    require(!hybrid.step(invalid, dt, settings), "non-finite anchor input is rejected");
    require(hybrid.positions().size() == before.size() &&
            distance(hybrid.positions()[1], before[1]) == 0.0F,
            "invalid input leaves last valid output unchanged");

    auto overflow = rest;
    overflow[0].x = std::numeric_limits<float>::max();
    require(!hybrid.step(overflow, dt, settings),
            "finite anchors that overflow derived velocity are rejected");
    require(distance(hybrid.positions()[1], before[1]) == 0.0F,
            "rejected derived motion preserves the last valid output");

    hybrid.reset(rest);
    require(hybrid.positions().size() == rest.size(), "reset restores the supplied pose");
    require(distance(hybrid.positions()[2], rest[2]) < 1.0e-6F,
            "reset clears all spring offsets");

    std::cout << "Hybrid exact-spring chain prototype passed 1200 frames. "
              << "Max segment-length error: " << max_length_error
              << "; max offset: " << max_offset << '\n'
              << "Production baseline max segment-length error: "
              << production_max_length_error << "; max offset: " << production_max_offset << '\n';
    return 0;
}
