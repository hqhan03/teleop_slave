#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace teleop_slave {

class JointTargetSmoother {
public:
    static constexpr std::size_t kNumJoints = 6;
    using JointArray = std::array<double, kNumJoints>;

    struct UpdateResult {
        JointArray filtered_target_deg{};
        bool held_due_to_invalid_dt{false};
    };

    void configure(const JointArray& deadband_deg,
                   const JointArray& velocity_limit_deg_s) {
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            deadband_deg_[i] = std::max(0.0, deadband_deg[i]);
            velocity_limit_deg_s_[i] = std::max(0.0, velocity_limit_deg_s[i]);
        }
    }

    void reset(const JointArray& initial_target_deg) {
        filtered_target_deg_ = initial_target_deg;
        initialized_ = true;
    }

    UpdateResult update(const JointArray& raw_target_deg, double dt_sec) {
        if (!initialized_) {
            reset(raw_target_deg);
        }

        UpdateResult result;
        result.filtered_target_deg = filtered_target_deg_;

        if (dt_sec <= 1e-4) {
            result.held_due_to_invalid_dt = true;
            return result;
        }

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            double desired_deg = raw_target_deg[i];
            if (std::abs(desired_deg - filtered_target_deg_[i]) < deadband_deg_[i]) {
                desired_deg = filtered_target_deg_[i];
            }

            const double max_step_deg = velocity_limit_deg_s_[i] * dt_sec;
            const double requested_step_deg = desired_deg - filtered_target_deg_[i];
            const double applied_step_deg = (max_step_deg > 0.0)
                ? std::clamp(requested_step_deg, -max_step_deg, max_step_deg)
                : requested_step_deg;
            filtered_target_deg_[i] += applied_step_deg;
        }

        result.filtered_target_deg = filtered_target_deg_;
        return result;
    }

    const JointArray& filtered_target_deg() const {
        return filtered_target_deg_;
    }

private:
    JointArray deadband_deg_{};
    JointArray velocity_limit_deg_s_{};
    JointArray filtered_target_deg_{};
    bool initialized_{false};
};

}  // namespace teleop_slave
