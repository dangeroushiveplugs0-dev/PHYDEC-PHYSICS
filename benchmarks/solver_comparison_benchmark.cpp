#include "phydec/experimental/exact_spring.hpp"
#include "phydec/secondary_motion.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

using Clock = std::chrono::steady_clock;

int main() {
    // This is a throughput microbenchmark, not a quality verdict: the models
    // differ (exact independent oscillators vs constrained two-point Verlet chains).
    // The two runs use equal counts and fixed work, and report timings only.
    constexpr std::size_t count = 20000;
    constexpr int frames = 600;
    constexpr float dt = 1.0F / 60.0F;

    phydec::experimental::ExactSpringSettings exact_settings;
    exact_settings.frequency_hz = 1.9F;
    exact_settings.damping_ratio = 0.14F;
    std::vector<phydec::experimental::ExactSpringState> exact(count);
    for (std::size_t i = 0; i < count; ++i) {
        exact[i].offset = {static_cast<float>(i % 13) * 0.01F, 0.2F, 0.0F};
        exact[i].velocity = {0.1F, -0.05F, 0.0F};
    }
    volatile float exact_sink = 0.0F;
    const auto exact_start = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (auto& state : exact) {
            phydec::experimental::step_exact_spring(state, dt, exact_settings);
            exact_sink = exact_sink + state.offset.y;
        }
    }
    const auto exact_end = Clock::now();

    phydec::SecondaryMotionSettings verlet_settings;
    verlet_settings.fixed_delta_seconds = dt;
    verlet_settings.constraint_iterations = 1;
    std::vector<phydec::SecondaryMotionChain> chains;
    chains.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        chains.emplace_back(std::vector<phydec::Vec3>{{0.0F, 0.0F, 0.0F},
            {static_cast<float>(i % 13) * 0.01F, 0.2F, 0.0F}});
    }
    volatile float verlet_sink = 0.0F;
    const auto verlet_start = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (auto& chain : chains) {
            chain.step(verlet_settings);
            verlet_sink = verlet_sink + chain.points().back().position.y;
        }
    }
    const auto verlet_end = Clock::now();

    const auto exact_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        exact_end - exact_start).count();
    const auto verlet_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        verlet_end - verlet_start).count();
    const double updates = static_cast<double>(count) * frames;
    std::cout << "Experimental spring integration microbenchmark (Release recommended)\n"
              << "states=" << count << " frames=" << frames << " dt=" << dt << '\n'
              << "exact oscillator ns/update=" << static_cast<double>(exact_ns) / updates << '\n'
              << "PHYDEC two-point chain ns/update=" << static_cast<double>(verlet_ns) / updates << '\n'
              << "timing is informational; models do different work and are not quality-equivalent\n"
              << "sinks=" << exact_sink << ',' << verlet_sink << '\n';
    if (!std::isfinite(exact_sink) || !std::isfinite(verlet_sink)) return 1;
    return 0;
}
