#pragma once

#include <vector>
#include "phydec/math/vec3.hpp"

namespace phydec {

struct MotionPoint {
    Vec3 position{};
    Vec3 previous_position{};
    Vec3 anchor{};
    float segment_length{1.0F};
    float stiffness{0.25F};
    float damping{0.08F};
    float mass{1.0F};
};

struct SecondaryMotionSettings {
    Vec3 gravity{0.0F, 0.0F, -9.81F};
    Vec3 external_acceleration{};
    float fixed_delta_seconds{1.0F / 60.0F};
    unsigned constraint_iterations{4};
};

// Small deterministic baseline for bone-chain secondary motion.
// Call set_anchors() each frame with the animated pose, then step().
class SecondaryMotionChain {
public:
    explicit SecondaryMotionChain(std::vector<Vec3> initial_positions = {});

    void set_anchors(const std::vector<Vec3>& animated_positions);
    void step(const SecondaryMotionSettings& settings);

    [[nodiscard]] const std::vector<MotionPoint>& points() const noexcept { return points_; }
    [[nodiscard]] bool valid() const noexcept;

private:
    std::vector<MotionPoint> points_;
};

} // namespace phydec
