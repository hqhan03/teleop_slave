#include <cmath>

#include <gtest/gtest.h>

#include "teleop_slave/kalman_filter.hpp"

TEST(KalmanFilter1D, RejectsSingleSpikeMeasurement) {
    teleop_slave::KalmanFilter1D filter;
    filter.configure(0.5, 2.5e-5);
    filter.reset(0.0);

    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(filter.filter(0.0, 0.02), 0.0, 1e-6);
    }

    const double spike_response = filter.filter(0.02, 0.02);
    EXPECT_LT(spike_response, 0.015);

    const double recovery_response = filter.filter(0.0, 0.02);
    EXPECT_LT(std::abs(recovery_response), std::abs(spike_response));
}

TEST(KalmanFilter1D, TracksSustainedMotionWithoutLargeLag) {
    teleop_slave::KalmanFilter1D filter;
    filter.configure(0.5, 2.5e-5);
    filter.reset(0.0);

    double output = 0.0;
    for (int i = 1; i <= 20; ++i) {
        output = filter.filter(0.002 * static_cast<double>(i), 0.02);
    }

    EXPECT_GT(output, 0.02);
    EXPECT_NEAR(output, 0.04, 0.003);
}
