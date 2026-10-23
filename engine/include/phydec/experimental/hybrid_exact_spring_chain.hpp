#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "phydec/experimental/exact_spring.hpp"

namespace phydec::experimental {

// Experimental combination of closed-form spring offsets and positional
// distance constraints. It is intentionally separate from SecondaryMotionChain:
// collision, rotation-aware local frames, and per-bone settings are not yet
// implemented here. Do not treat this prototype as a production replacement.
class HybridExactSpringChain {
public:
    explicit HybridExactSpringChain(std::vector<Vec3> initial_anchors = {})
        : previous_anchors_(initial_anchors),
          previous_anchor_velocities_(initial_anchors.size()),
          output_positions_(initial_anchors), springs_(initial_anchors.size()) {
        initialized_ = !initial_anchors.empty() && anchors_are_finite(initial_anchors);
        if (initialized_) {
            rebuild_rest_lengths();
        } else {
            previous_anchors_.clear();
            previous_anchor_velocities_.clear();
            output_positions_.clear();
            springs_.clear();
        }
    }

    void reset(const std::vector<Vec3>& anchors = {}) {
        previous_anchors_ = anchors;
        output_positions_ = anchors;
        springs_.assign(anchors.size(), ExactSpringState{});
        previous_anchor_velocities_.assign(anchors.size(), Vec3{});
        rebuild_rest_lengths();
        initialized_ = !anchors.empty() && anchors_are_finite(anchors);
        if (!initialized_) {
            previous_anchors_.clear();
            output_positions_.clear();
            springs_.clear();
            previous_anchor_velocities_.clear();
            rest_lengths_.clear();
        }
    }

    // Returns false for invalid input and leaves the last valid output intact.
    [[nodiscard]] bool step(const std::vector<Vec3>& animated_anchors, float dt,
                            const ExactSpringSettings& spring_settings,
                            unsigned constraint_iterations = 6) {
        if (animated_anchors.empty() || !anchors_are_finite(animated_anchors) ||
            !(dt > 0.0F) || !std::isfinite(dt) ||
            !std::isfinite(spring_settings.frequency_hz) ||
            !std::isfinite(spring_settings.damping_ratio) ||
            !std::isfinite(spring_settings.relative_size) ||
            !is_finite(spring_settings.gravity)) {
            return false;
        }

        if (!initialized_ || animated_anchors.size() != previous_anchors_.size()) {
            reset(animated_anchors);
            return initialized_;
        }

        const std::size_t count = animated_anchors.size();
        // Work on a private candidate state. A failed update must not partially
        // advance hidden spring state while leaving the public output unchanged.
        auto next_springs = springs_;
        std::vector<Vec3> anchor_velocities(count);
        anchor_velocities[0] = (animated_anchors[0] - previous_anchors_[0]) / dt;
        if (!is_finite(anchor_velocities[0])) return false;
        std::vector<Vec3> candidate(count);
        candidate[0] = animated_anchors[0];

        for (std::size_t i = 1; i < count; ++i) {
            anchor_velocities[i] = (animated_anchors[i] - previous_anchors_[i]) / dt;
            const Vec3 parent_velocity = (animated_anchors[i - 1] - previous_anchors_[i - 1]) / dt;
            const Vec3 parent_acceleration = (parent_velocity - previous_anchor_velocities_[i - 1]) / dt;
            if (!is_finite(anchor_velocities[i]) || !is_finite(parent_velocity) ||
                !is_finite(parent_acceleration)) {
                return false;
            }
            step_exact_spring(next_springs[i], dt, spring_settings, parent_acceleration);
            candidate[i] = animated_anchors[i] + next_springs[i].offset;
        }

        // Preserve the animated root and project the spring candidates onto
        // their original segment lengths. This is a small PBD-style correction,
        // not a full mass-weighted rigid-body or soft-body constraint solver.
        for (unsigned iteration = 0; iteration < constraint_iterations; ++iteration) {
            candidate[0] = animated_anchors[0];
            for (std::size_t i = 1; i < count; ++i) {
                const Vec3 delta = candidate[i] - candidate[i - 1];
                const float current_length = length(delta);
                const float rest_length = rest_lengths_[i - 1];
                if (!(current_length > 1.0e-7F) || !std::isfinite(current_length) ||
                    !std::isfinite(rest_length)) {
                    continue;
                }
                const Vec3 correction = delta * ((current_length - rest_length) / current_length);
                if (i == 1) {
                    candidate[i] -= correction;
                } else {
                    candidate[i - 1] += correction * 0.5F;
                    candidate[i] -= correction * 0.5F;
                }
            }
        }
        candidate[0] = animated_anchors[0];

        if (!anchors_are_finite(candidate)) return false;

        // Feed constraint corrections back into spring offsets so the next
        // analytical step starts from the constrained configuration.
        for (std::size_t i = 1; i < count; ++i) {
            next_springs[i].offset = candidate[i] - animated_anchors[i];
        }
        springs_ = std::move(next_springs);
        previous_anchors_ = animated_anchors;
        previous_anchor_velocities_ = std::move(anchor_velocities);
        output_positions_ = std::move(candidate);
        return true;
    }

    [[nodiscard]] const std::vector<Vec3>& positions() const noexcept {
        return output_positions_;
    }

private:
    static bool anchors_are_finite(const std::vector<Vec3>& values) noexcept {
        for (const Vec3& value : values) if (!is_finite(value)) return false;
        return true;
    }

    void rebuild_rest_lengths() {
        rest_lengths_.clear();
        if (previous_anchors_.size() < 2) return;
        rest_lengths_.reserve(previous_anchors_.size() - 1);
        for (std::size_t i = 1; i < previous_anchors_.size(); ++i) {
            rest_lengths_.push_back(length(previous_anchors_[i] - previous_anchors_[i - 1]));
        }
    }

    std::vector<Vec3> previous_anchors_{};
    std::vector<Vec3> previous_anchor_velocities_{};
    std::vector<Vec3> output_positions_{};
    std::vector<ExactSpringState> springs_{};
    std::vector<float> rest_lengths_{};
    bool initialized_{false};
};

} // namespace phydec::experimental
