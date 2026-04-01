#pragma once

#include <cmath>

namespace teleop_slave {

/// Second-order Butterworth low-pass IIR filter.
///
/// Coefficients are computed via the bilinear transform.  The filter runs as a
/// Direct-Form II transposed structure for numerical stability.
class ButterworthFilter2 {
public:
    ButterworthFilter2() = default;

    /// Compute filter coefficients for the given cutoff and sample rate.
    void configure(double cutoff_hz, double sample_rate_hz) {
        const double omega = 2.0 * M_PI * cutoff_hz / sample_rate_hz;
        const double s = std::sin(omega);
        const double c = std::cos(omega);
        const double alpha = s / std::sqrt(2.0);  // Q = 1/sqrt(2) for Butterworth

        const double a0 = 1.0 + alpha;
        b0_ = ((1.0 - c) / 2.0) / a0;
        b1_ = (1.0 - c) / a0;
        b2_ = b0_;
        a1_ = (-2.0 * c) / a0;
        a2_ = (1.0 - alpha) / a0;
    }

    /// Reset filter state for bumpless initialisation.
    void reset(double initial_value) {
        // Set state so that a constant input of initial_value produces
        // initial_value at the output (steady-state solution).
        const double y_ss = initial_value;
        const double denom = 1.0 + a1_ + a2_;
        const double x_ss = (denom != 0.0) ? y_ss / ((b0_ + b1_ + b2_)) : y_ss;
        // Direct-Form II Transposed steady-state:
        //   z1 = (b1 - a1) * x_ss  +  (b2 - a2) * x_ss  (from difference eq.)
        //   but simplest bumpless start is to prime both delays with x_ss.
        z1_ = x_ss * (b1_ - a1_) + x_ss * (b2_ - a2_);
        z2_ = x_ss * (b2_ - a2_);
        initialized_ = true;
    }

    /// Process one sample and return the filtered output.
    double filter(double x) {
        if (!initialized_) {
            reset(x);
        }
        const double y = b0_ * x + z1_;
        z1_ = b1_ * x - a1_ * y + z2_;
        z2_ = b2_ * x - a2_ * y;
        return y;
    }

private:
    // Coefficients (normalised by a0).
    double b0_{1.0}, b1_{0.0}, b2_{0.0};
    double a1_{0.0}, a2_{0.0};

    // Direct-Form II Transposed delay elements.
    double z1_{0.0}, z2_{0.0};

    bool initialized_{false};
};

}  // namespace teleop_slave
