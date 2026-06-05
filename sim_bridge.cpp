// sim_bridge.cpp
#include "compat/Arduino.h"       // ← must be first, before everything
#include "compat/Wire.h"          // ← second
#include "sim_bridge.h"
#include "imu_sim.h"
#include "flight/sensfusion6.h"
#include "flight/controller_pid.h"
#include "flight/flight_types.h"
#include "power/power_distrib.h"

#define SIM_DT (1.0f / 40.0f)   // matches agent_hz=40 in Python

// ── Your real class instances ─────────────────────────────────────────────────
static SensFusion6        s_fusion;
static ControllerPid      s_pid;
static PowerDistribution  s_mixer;

// ─────────────────────────────────────────────────────────────────────────────
void sim_init(void) {
    s_fusion.init(0.0f, 0.0f, 0.0f);  // start level
    s_pid.init(SIM_DT);
}

void sim_reset(void) {
    s_fusion.init(0.0f, 0.0f, 0.0f);
    s_fusion.resetYaw();
    s_pid.reset();
}

// ─── Phase 2: fusion only ────────────────────────────────────────────────────
void sim_update_fusion(const SimInput* in, SimAttitude* att_out) {
    IMUData imu = imu_from_obs(in->obs, in->accel_ms2);

    if (!imu.isHealthy) {
        // Bad frame — return last known attitude, skip filter update
        AttitudeDeg att = s_fusion.attitude();
        att_out->roll   = att.roll;
        att_out->pitch  = att.pitch;
        att_out->yaw    = att.yaw;
        return;
    }

    // YOUR real complementary filter — same code that runs on ESP32
    s_fusion.update(imu.gx, imu.gy, imu.gz,
                    imu.ax, imu.ay, imu.az,
                    SIM_DT);

    AttitudeDeg att = s_fusion.attitude();
    att_out->roll   = att.roll;
    att_out->pitch  = att.pitch;
    att_out->yaw    = att.yaw;
}

// ─── Phase 3+: full pipeline (filled in at phase 3) ─────────────────────────
// Replace the placeholder sim_update_full with this:
void sim_update_full(const SimInput* in,
                     float    target_roll,
                     float    target_pitch,
                     float    target_yaw_rate,
                     uint8_t  base_thrust,
                     SimMotors* motors_out,
                     SimAttitude* att_out) {

    // ── Step 1: Convert PyFlyt obs → your IMUData struct ─────────────────────
    IMUData imu = imu_from_obs(in->obs, in->accel_ms2);

    if (!imu.isHealthy) {
        // Unhealthy IMU → cut motors, same as your emergencyStop()
        for (int i = 0; i < 4; i++) motors_out->m[i] = 0.0f;
        return;
    }

    // ── Step 2: YOUR sensor fusion ────────────────────────────────────────────
    s_fusion.update(imu.gx, imu.gy, imu.gz,
                    imu.ax, imu.ay, imu.az,
                    SIM_DT);

    AttitudeDeg att = s_fusion.attitude();
    att_out->roll   = att.roll;
    att_out->pitch  = att.pitch;
    att_out->yaw    = att.yaw;

    // ── Step 3: Build YOUR Setpoint struct ────────────────────────────────────
    Setpoint sp;
    sp.attitudeDeg.roll          = target_roll;
    sp.attitudeDeg.pitch         = target_pitch;
    sp.attitudeDeg.yaw           = 0.0f;
    sp.attitudeRateDps.roll      = 0.0f;
    sp.attitudeRateDps.pitch     = 0.0f;
    sp.attitudeRateDps.yaw       = target_yaw_rate;

    // ── Step 4: Gyro rates for D-term ─────────────────────────────────────────
    // Your PID uses updateWithRate() — D-term reads directly from gyro
    // This is cleaner than differentiating the fused angle
    AttitudeRateDps rates;
    rates.roll  = imu.gx;   // already in deg/s from imu_from_obs()
    rates.pitch = imu.gy;
    rates.yaw   = imu.gz;

    // ── Step 5: YOUR PID controller ───────────────────────────────────────────
    s_pid.setDt(SIM_DT);
    ControlOutput ctrl = s_pid.update(sp, att, rates);

    // ── Step 6: YOUR PowerDistribution mixer ──────────────────────────────────
    MotorMix mix = s_mixer.mixX(base_thrust, ctrl);

    // ── Step 7: Normalize 0-255 → 0.0-1.0 for PyFlyt ─────────────────────────
    // MOTOR_MAX = 200, so divide by 200 not 255
    motors_out->m[0] = mix.m1 / (float)MOTOR_MAX;
    motors_out->m[1] = mix.m2 / (float)MOTOR_MAX;
    motors_out->m[2] = mix.m3 / (float)MOTOR_MAX;
    motors_out->m[3] = mix.m4 / (float)MOTOR_MAX;
}