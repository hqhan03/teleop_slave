#pragma once

#include <array>
#include <memory>

#include <ruckig/ruckig.hpp>

namespace teleop_slave {

/// Jerk-limited online trajectory generator for 6-DOF joint streaming.
/// Wraps Ruckig to produce C2-continuous joint trajectories at the servo loop
/// rate (e.g. 125 Hz).  The caller updates the target asynchronously (from IK
/// at 50 Hz) and samples the trajectory each servo cycle via update().
class RuckigJointTrajectoryGenerator {
public:
    static constexpr std::size_t kNumJoints = 6;
    using JointArray = std::array<double, kNumJoints>;

    struct Config {
        double cycle_time_sec{0.008};
        JointArray max_velocity_deg_s{};
        JointArray max_acceleration_deg_s2{};
        JointArray max_jerk_deg_s3{};
    };

    void configure(const Config& config) {
        config_ = config;
    }

    /// Reset to a known position with zero velocity and acceleration.
    /// Call on servo session start, clutch engage, etc.
    void reset(const JointArray& current_position_deg) {
        ruckig_ = std::make_unique<ruckig::Ruckig<kNumJoints>>(config_.cycle_time_sec);
        input_ = ruckig::InputParameter<kNumJoints>{};

        for (std::size_t i = 0; i < kNumJoints; ++i) {
            input_.current_position[i] = current_position_deg[i];
            input_.current_velocity[i] = 0.0;
            input_.current_acceleration[i] = 0.0;
            input_.target_position[i] = current_position_deg[i];
            input_.target_velocity[i] = 0.0;
            input_.target_acceleration[i] = 0.0;
            input_.max_velocity[i] = config_.max_velocity_deg_s[i];
            input_.max_acceleration[i] = config_.max_acceleration_deg_s2[i];
            input_.max_jerk[i] = config_.max_jerk_deg_s3[i];
        }

        initialized_ = true;
        error_logged_ = false;
    }

    /// Update the goal target position.  Target velocity/acceleration remain
    /// zero (i.e. come to rest at the target).  Safe to call at any rate.
    void setTarget(const JointArray& target_position_deg) {
        for (std::size_t i = 0; i < kNumJoints; ++i) {
            input_.target_position[i] = target_position_deg[i];
        }
    }

    /// Advance the trajectory by one cycle and return the next commanded
    /// position.  Must be called at the configured cycle rate (e.g. 125 Hz).
    JointArray update() {
        JointArray result{};

        if (!initialized_ || !ruckig_) {
            return result;
        }

        const auto status = ruckig_->update(input_, output_);

        if (status == ruckig::Result::Working ||
            status == ruckig::Result::Finished) {
            output_.pass_to_input(input_);
            std::copy(output_.new_position.begin(),
                      output_.new_position.end(),
                      result.begin());
            error_logged_ = false;
        } else {
            // On error, hold the current position
            std::copy(input_.current_position.begin(),
                      input_.current_position.end(),
                      result.begin());
            if (!error_logged_) {
                error_logged_ = true;
            }
        }

        return result;
    }

    bool initialized() const { return initialized_; }
    bool lastUpdateHadError() const { return error_logged_; }

private:
    Config config_{};
    std::unique_ptr<ruckig::Ruckig<kNumJoints>> ruckig_;
    ruckig::InputParameter<kNumJoints> input_{};
    ruckig::OutputParameter<kNumJoints> output_{};
    bool initialized_{false};
    bool error_logged_{false};
};

}  // namespace teleop_slave
