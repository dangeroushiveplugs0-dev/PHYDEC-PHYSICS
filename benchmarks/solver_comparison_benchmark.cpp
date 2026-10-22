#include "phydec/experimental/exact_spring.hpp"
#include "phydec/secondary_motion.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

using Clock = std::chrono::steady_clock;

namespace {
void step_semi_implicit_euler(phydec::experimental::ExactSpringState& state, float dt,
                              const phydec::experimental::ExactSpringSettings& settings) {
    constexpr float tau = 6.2831853071795864769F;
    const float omega = tau * settings.frequency_hz /
                        std::sqrt(std::max(settings.relative_size, 1.0e-4F));
    const float zeta = settings.damping_ratio /
                       std::sqrt(std::max(settings.relative_size, 1.0e-4F));
    const phydec::Vec3 acceleration = settings.gravity -
        state.offset * (omega * omega) -
        state.velocity * (2.0F * zeta * omega);
    state.velocity += acceleration * dt;
    state.offset += state.velocity * dt;
}
}

int main() {
    constexpr std::size_t count = 20000;
    constexpr int frames = 600;
    constexpr float dt = 1.0F / 60.0F;
    phydec::experimental::ExactSpringSettings settings;
    settings.frequency_hz = 1.9F;
    settings.damping_ratio = 0.14F;

    std::vector<phydec::experimental::ExactSpringState> exact(count);
    std::vector<phydec::experimental::ExactSpringState> approximate(count);
    for (std::size_t i = 0; i < count; ++i) {
        const phydec::Vec3 offset{static_cast<float>(i % 13) * 0.01F, 0.2F, 0.0F};
        const phydec::Vec3 velocity{0.1F, -0.05F, 0.0F};
        exact[i] = {offset, velocity};
        approximate[i] = {offset, velocity};
    }

    volatile float exact_sink = 0.0F;
    const auto exact_start = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (auto& state : exact) {
            phydec::experimental::step_exact_spring(state, dt, settings);
            exact_sink = exact_sink + state.offset.y;
        }
    }
    const auto exact_end = Clock::now();

    volatile float approximate_sink = 0.0F;
    const auto approximate_start = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (auto& state : approximate) {
            step_semi_implicit_euler(state, dt, settings);
            approximate_sink = approximate_sink + state.offset.y;
        }
    }
    const auto approximate_end = Clock::now();

    // Context-only workload: the current PHYDEC two-point chain also performs
    // distance-constraint work, so it is not the matched-equation comparison.
    phydec::SecondaryMotionSettings verlet_settings;
    verlet_settings.fixed_delta_seconds = dt;
    verlet_settings.constraint_iterations = 1;
    std::vector<phydec::SecondaryMotionChain> chains;
    chains.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        chains.emplace_back(std::vector<phydec::Vec3>{{0.0F, 0.0F, 0.0F},
            {static_cast<float>(i % 13) * 0.01F, 0.2F, 0.0F}});
    }
    volatile float chain_sink = 0.0F;
    const auto chain_start = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
        for (auto& chain : chains) {
            chain.step(verlet_settings);
            chain_sink = chain_sink + chain.points().back().position.y;
        }
    }
    const auto chain_end = Clock::now();

    const auto elapsed_ns = [](auto start, auto end) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    };
    const double updates = static_cast<double>(count) * frames;
    const double exact_ns = static_cast<double>(elapsed_ns(exact_start, exact_end)) / updates;
    const double approximate_ns = static_cast<double>(elapsed_ns(approximate_start, approximate_end)) / updates;
    const double chain_ns = static_cast<double>(elapsed_ns(chain_start, chain_end)) / updates;
    std::cout << "Solver comparison microbenchmark (Release recommended)\n"
              << "states=" << count << " frames=" << frames << " dt=" << dt << '\n'
              << "MATCHED EQUATION: exact oscillator ns/update=" << exact_ns << '\n'
              << "MATCHED EQUATION: semi-implicit Euler ns/update=" << approximate_ns << '\n'
              << "CONTEXT ONLY: PHYDEC two-point constrained chain ns/update=" << chain_ns << '\n'
              << "Matched equation integrates the same spring ODE; chain context does different work. "
                 "Timing is informational, not a CI threshold.\n"
              << "sinks=" << exact_sink << ',' << approximate_sink << ',' << chain_sink << '\n';
    if (!std::isfinite(exact_sink) || !std::isfinite(approximate_sink) ||
        !std::isfinite(chain_sink)) return 1;
    return 0;
}
