#pragma once

#include <algorithm>
#include <cmath>

namespace teleop_slave {

/// Constant-velocity 1D Kalman filter for smoothing position measurements.
class KalmanFilter1D {
public:
    KalmanFilter1D() = default;

    /// Configure the continuous-time acceleration noise and measurement variance.
    void configure(double process_noise, double measurement_noise) {
        process_noise_ = std::max(process_noise, kMinNoise);
        measurement_noise_ = std::max(measurement_noise, kMinNoise);
    }

    /// Reset the state to a known position with zero velocity.
    void reset(double position, double velocity = 0.0, double covariance = 1e-3) {
        position_ = position;
        velocity_ = velocity;
        covariance = std::max(covariance, kMinNoise);
        p00_ = covariance;
        p01_ = 0.0;
        p10_ = 0.0;
        p11_ = covariance;
        initialized_ = true;
    }

    /// Filter one position sample using the given timestep in seconds.
    double filter(double measurement, double dt_sec) {
        if (!initialized_) {
            reset(measurement);
            return position_;
        }

        const double dt = std::clamp(dt_sec, 1e-4, 0.2);

        // Predict using a constant-velocity model.
        position_ += velocity_ * dt;

        const double q00 = process_noise_ * dt * dt * dt * dt * 0.25;
        const double q01 = process_noise_ * dt * dt * dt * 0.5;
        const double q11 = process_noise_ * dt * dt;

        const double pred_p00 = p00_ + dt * (p10_ + p01_) + dt * dt * p11_ + q00;
        const double pred_p01 = p01_ + dt * p11_ + q01;
        const double pred_p10 = p10_ + dt * p11_ + q01;
        const double pred_p11 = p11_ + q11;

        const double innovation = measurement - position_;
        const double innovation_cov = pred_p00 + measurement_noise_;
        const double k0 = pred_p00 / innovation_cov;
        const double k1 = pred_p10 / innovation_cov;

        position_ += k0 * innovation;
        velocity_ += k1 * innovation;

        p00_ = std::max((1.0 - k0) * pred_p00, kMinNoise);
        p01_ = (1.0 - k0) * pred_p01;
        p10_ = p01_;
        p11_ = std::max(pred_p11 - k1 * pred_p01, kMinNoise);
        return position_;
    }

    double position() const { return position_; }
    double velocity() const { return velocity_; }

private:
    static constexpr double kMinNoise = 1e-9;

    double process_noise_{1.0};
    double measurement_noise_{1e-4};

    double position_{0.0};
    double velocity_{0.0};

    double p00_{1e-3};
    double p01_{0.0};
    double p10_{0.0};
    double p11_{1e-3};

    bool initialized_{false};
};

}  // namespace teleop_slave
