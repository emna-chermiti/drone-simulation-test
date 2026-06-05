// imu_sim.h
// Bypasses I2C — injects PyFlyt observation data directly into your IMUData struct
#pragma once
#include "compat/Arduino.h"
#include "imu.h"
#include <cmath>

// obs layout confirmed from PyFlyt QuadX-Hover-v4:
//   [0..2]  angular velocity rad/s  → your code needs deg/s
//   [3..6]  quaternion xyzw         → ground truth for comparison
//   [7..9]  linear velocity m/s
//   [10..12] position m
//   [13..15] target position m

inline IMUData imu_from_obs(const float* obs, const float* noisy_accel_ms2) {
    IMUData d = {};

    // ── Gyro: rad/s → deg/s ───────────────────────────────────────────────────
    // PyFlyt gives rad/s, your sensfusion6::update() takes deg/s
    d.gx = obs[0] * RAD_TO_DEG;
    d.gy = obs[1] * RAD_TO_DEG;
    d.gz = obs[2] * RAD_TO_DEG;

    // ── Accel: m/s² → G ───────────────────────────────────────────────────────
    // imu_simulator.py outputs m/s², your sensfusion6::update() takes G units
    d.ax = noisy_accel_ms2[0] / 9.81f;
    d.ay = noisy_accel_ms2[1] / 9.81f;
    d.az = noisy_accel_ms2[2] / 9.81f;

    // ── Health check — mirrors your real readIMU() sanity check ───────────────
    float mag = sqrtf(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
    d.isHealthy = (mag >= 0.3f && mag <= 3.0f);

    return d;
}