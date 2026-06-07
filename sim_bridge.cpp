#include "compat/Arduino.h"
#include "sim_bridge.h"
#include "imu_sim.h"
#include "flight/sensfusion6.h"
#include "flight/controller_pid.h"
#include "flight/flight_types.h"
#include "power/power_distrib.h"

#define SIM_DT (1.0f / 40.0f)

static SensFusion6        s_fusion;
static ControllerPid      s_pid;
static PowerDistribution  s_mixer;

void sim_init(void) {
    s_fusion.init(0.0f, 0.0f, 0.0f);
    s_pid.init(SIM_DT);
}

void sim_reset(void) {
    s_fusion.init(0.0f, 0.0f, 0.0f);
    s_fusion.resetYaw();
    s_pid.reset();
}

void sim_update_fusion(const SimInput* in, SimAttitude* att_out) {
    IMUData imu = imu_from_obs(in->obs, in->accel_ms2);

    if (!imu.isHealthy) {
        AttitudeDeg att = s_fusion.attitude();
        att_out->roll   = att.roll;
        att_out->pitch  = att.pitch;
        att_out->yaw    = att.yaw;
        return;
    }

    s_fusion.update(imu.gx, imu.gy, imu.gz,
                    imu.ax, imu.ay, imu.az,
                    SIM_DT);

    AttitudeDeg att = s_fusion.attitude();
    att_out->roll   = att.roll;
    att_out->pitch  = att.pitch;
    att_out->yaw    = att.yaw;
}

void sim_update_full(const SimInput* in,
                     float    target_roll,
                     float    target_pitch,
                     float    target_yaw_rate,
                     uint8_t  base_thrust,
                     SimMotors* motors_out,
                     SimAttitude* att_out) {

    IMUData imu = imu_from_obs(in->obs, in->accel_ms2);
    if (!imu.isHealthy) {
        for (int i = 0; i < 4; i++) motors_out->m[i] = 0.0f;
        return;
    }

    s_fusion.update(imu.gx, imu.gy, imu.gz,
                    imu.ax, imu.ay, imu.az,
                    SIM_DT);

    AttitudeDeg att = s_fusion.attitude();
    att_out->roll   = att.roll;
    att_out->pitch  = att.pitch;
    att_out->yaw    = att.yaw;

    // ── Altitude PD ───────────────────────────────────────────────────────────
    float pos_z   = in->obs[10];
    float tgt_z   = in->obs[13];
    float vel_z   = in->obs[9];
    float alt_err = tgt_z - pos_z;

    const float KP_ALT = 6.0f;
    const float KD_ALT = 5.0f;

    float thrust_f = (float)base_thrust
                   + KP_ALT * alt_err
                   - KD_ALT * vel_z;

    if (thrust_f > MOTOR_MAX) thrust_f = MOTOR_MAX;
    if (thrust_f < MOTOR_MIN) thrust_f = MOTOR_MIN;
    uint8_t thrust = (uint8_t)thrust_f;

    // ── Attitude PD ───────────────────────────────────────────────────────────
    Setpoint sp;
    sp.attitudeDeg.roll          = target_roll;
    sp.attitudeDeg.pitch         = target_pitch;
    sp.attitudeDeg.yaw           = 0.0f;
    sp.attitudeRateDps.roll      = 0.0f;
    sp.attitudeRateDps.pitch     = 0.0f;
    sp.attitudeRateDps.yaw       = target_yaw_rate;

    // ── Re-enable D-term with scaled gyro rates ───────────────────────────────
    // 0.1 factor accounts for 40Hz vs 500Hz — gyro rates are 12.5x
    // larger per step at 40Hz, scale them down so D-term doesn't saturate
    AttitudeRateDps rates;
    rates.roll  = imu.gx * 0.1f;
    rates.pitch = imu.gy * 0.1f;
    rates.yaw   = imu.gz * 0.1f;

    s_pid.reset();
    ControlOutput ctrl = s_pid.update(sp, att, rates);

    const float GAIN_SCALE = 0.003f;
    ctrl.roll  *= GAIN_SCALE;
    ctrl.pitch *= GAIN_SCALE;
    ctrl.yaw   *= GAIN_SCALE;

    MotorMix mix = s_mixer.mixX(thrust, ctrl);

    motors_out->m[0] = mix.m1 / (float)MOTOR_MAX;
    motors_out->m[1] = mix.m2 / (float)MOTOR_MAX;
    motors_out->m[2] = mix.m3 / (float)MOTOR_MAX;
    motors_out->m[3] = mix.m4 / (float)MOTOR_MAX;
}