
#pragma once
#include "compat/Arduino.h"
#include "imu.h"
#include <cmath>

inline IMUData imu_from_obs(const float* obs, const float* noisy_accel_ms2) {
    IMUData d = {};

    d.gx = obs[0] * RAD_TO_DEG;
    d.gy = obs[1] * RAD_TO_DEG;
    d.gz = obs[2] * RAD_TO_DEG;

    d.ax = noisy_accel_ms2[0] / 9.81f;
    d.ay = noisy_accel_ms2[1] / 9.81f;
    d.az = noisy_accel_ms2[2] / 9.81f;

    float mag = sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
    d.isHealthy = (mag >= 0.3f && mag <= 3.0f);

    return d;
}