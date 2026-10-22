#pragma once

#include <algorithm>
#include <cmath>

#include "phydec/math/vec3.hpp"

namespace phydec::experimental {

// Isolated prototype based on the damped-oscillator equation documented by
// xloveee/jiggle-physics (BSD-3-Clause):
//   x'' = -w^2 * x - 2*zeta*w*x' + gravity - parent_acceleration
// This is experimental, not the production solver API. No reference source code
// is copied; this expresses the documented mathematical model in C++20.
struct ExactSpringSettings {
    float frequency_hz{1.9F};
    float damping_ratio{0.14F};
    float relative_size{1.0F};
    Vec3 gravity{0.0F, -2.7F, 0.0F};
};

struct ExactSpringState {
    Vec3 offset{};
    Vec3 velocity{};
};

inline void step_exact_spring(ExactSpringState& state, float dt,
                              const ExactSpringSettings& settings,
                              const Vec3& parent_acceleration = {}) noexcept {
    if (!(dt > 0.0F) || !std::isfinite(dt) ||
        !is_finite(state.offset) || !is_finite(state.velocity) ||
        !std::isfinite(settings.frequency_hz) ||
        !std::isfinite(settings.damping_ratio) ||
        !std::isfinite(settings.relative_size) ||
        !is_finite(settings.gravity) || !is_finite(parent_acceleration)) {
        return;
    }

    constexpr float tau = 6.2831853071795864769F;
    const float size = std::max(settings.relative_size, 1.0e-4F);
    const float frequency = std::max(settings.frequency_hz, 1.0e-4F);
    const float omega = std::max(tau * frequency / std::sqrt(size), 1.0e-3F);
    const float zeta = std::max(settings.damping_ratio / std::sqrt(size), 0.0F);
    const float omega_squared = omega * omega;
    const float zeta_omega = zeta * omega;
    const Vec3 forcing = settings.gravity + parent_acceleration * -1.0F;
    const Vec3 equilibrium = forcing / omega_squared;
    const Vec3 x = state.offset - equilibrium;
    const Vec3 v = state.velocity;
    const float discriminant = zeta * zeta - 1.0F;

    if (discriminant < -1.0e-5F) {
        const float decay = std::exp(-zeta_omega * dt);
        const float damped_omega = omega * std::sqrt(1.0F - zeta * zeta);
        const float cosine = std::cos(damped_omega * dt);
        const float sine_over_omega = std::sin(damped_omega * dt) / damped_omega;
        state.offset = equilibrium +
            (x * cosine + (v + x * zeta_omega) * sine_over_omega) * decay;
        state.velocity =
            (v * cosine - (x * omega_squared + v * zeta_omega) * sine_over_omega) * decay;
    } else if (discriminant <= 1.0e-5F) {
        const float decay = std::exp(-omega * dt);
        const Vec3 c = v + x * omega;
        state.offset = equilibrium + (x + c * dt) * decay;
        state.velocity = (v - c * (omega * dt)) * decay;
    } else {
        const float root = omega * std::sqrt(discriminant);
        const float r1 = -zeta_omega + root;
        const float r2 = -zeta_omega - root;
        const float denominator = r1 - r2;
        const Vec3 a = (v - x * r2) / denominator;
        const Vec3 b = (x * r1 - v) / denominator;
        const float e1 = std::exp(r1 * dt);
        const float e2 = std::exp(r2 * dt);
        state.offset = equilibrium + a * e1 + b * e2;
        state.velocity = a * (r1 * e1) + b * (r2 * e2);
    }
}

/**
 * Experimental anchor-driven bone offset wrapper.
 *
 * Feed the animated anchor position once per simulation update. The class
 * estimates anchor acceleration from successive samples and returns the anchor
 * plus a damped spring offset. This is independent-bone motion: it does not
 * enforce lengths between bones and does not apply collision response.
 */
class ExactSpringBone {
public:
    void reset(const Vec3& anchor = {}) noexcept {
        state_ = {};
        previous_anchor_ = anchor;
        previous_anchor_velocity_ = {};
        initialized_ = is_finite(anchor);
    }

    [[nodiscard]] Vec3 update(const Vec3& animated_anchor, float dt,
                              const ExactSpringSettings& settings) noexcept {
        if (!is_finite(animated_anchor) || !(dt > 0.0F) || !std::isfinite(dt)) {
            return initialized_ ? previous_anchor_ + state_.offset : animated_anchor;
        }
        if (!initialized_) {
            reset(animated_anchor);
            return animated_anchor;
        }

        const Vec3 anchor_velocity = (animated_anchor - previous_anchor_) / dt;
        const Vec3 anchor_acceleration = (anchor_velocity - previous_anchor_velocity_) / dt;
        step_exact_spring(state_, dt, settings, anchor_acceleration);
        previous_anchor_ = animated_anchor;
        previous_anchor_velocity_ = anchor_velocity;
        return animated_anchor + state_.offset;
    }

    [[nodiscard]] const ExactSpringState& state() const noexcept { return state_; }

private:
    ExactSpringState state_{};
    Vec3 previous_anchor_{};
    Vec3 previous_anchor_velocity_{};
    bool initialized_{false};
};

} // namespace phydec::experimental
