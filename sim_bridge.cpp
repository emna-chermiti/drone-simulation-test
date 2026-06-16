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
    {
        float mag2 = imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az;
        imu.isHealthy = (mag2 >= 0.05f*0.05f && mag2 <= 8.0f*8.0f);
    }
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

    float pos_y   = in->obs[11];
    float tgt_y   = in->obs[13];
    float vel_y   = in->obs[8];
    float alt_err = tgt_y - pos_y;

    const float KP_ALT = 2.5f;
    const float KD_ALT = 2.0f;
    const float ALT_THRUST_MIN = 60.0f;  
    const float ALT_THRUST_MAX = 180.0f; 

    float thrust_f = (float)base_thrust
                   + (KP_ALT * alt_err)
                   - (KD_ALT * vel_y);

    if (thrust_f > ALT_THRUST_MAX) thrust_f = ALT_THRUST_MAX;
    if (thrust_f < ALT_THRUST_MIN) thrust_f = ALT_THRUST_MIN;
    uint8_t thrust = (uint8_t)thrust_f;

    Setpoint sp;
    sp.attitudeDeg.roll          = target_roll;
    sp.attitudeDeg.pitch         = target_pitch;
    sp.attitudeDeg.yaw           = 0.0f;
    sp.attitudeRateDps.roll      = 0.0f;
    sp.attitudeRateDps.pitch     = 0.0f;
    sp.attitudeRateDps.yaw       = target_yaw_rate;

    AttitudeRateDps rates;
    auto clampRate = [](float v) -> float {
        if (v >  360.0f) return  360.0f;
        if (v < -360.0f) return -360.0f;
        return v;
    };
    rates.roll  = clampRate(imu.gx * 0.1f);
    rates.pitch = clampRate(imu.gy * 0.1f);
    rates.yaw   = clampRate(imu.gz * 0.1f);

    ControlOutput ctrl = s_pid.update(sp, att, rates);

    const float GAIN_SCALE = 0.08f;
    ctrl.roll  *= GAIN_SCALE;
    ctrl.pitch *= GAIN_SCALE;
    ctrl.yaw   *= GAIN_SCALE;

    MotorMix mix = s_mixer.mixX(thrust, ctrl);

    motors_out->m[0] = mix.m1;
    motors_out->m[1] = mix.m2;
    motors_out->m[2] = mix.m3;
    motors_out->m[3] = mix.m4;

}

void sim_update_highlevel(const SimInput* in,
                          float    target_yaw_rate,
                          float    roll_cmd,
                          float    pitch_cmd,
                          uint8_t  base_thrust,
                          SimMotors* motors_out,
                          SimAttitude* att_out) {

    (void)target_yaw_rate; (void)roll_cmd; (void)pitch_cmd;

    IMUData imu = imu_from_obs(in->obs, in->accel_ms2);
    {
        float mag2 = imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az;
        imu.isHealthy = (mag2 >= 0.05f*0.05f && mag2 <= 8.0f*8.0f);
    }
    if (!imu.isHealthy) {
        motors_out->m[0] = 0.0f;
        motors_out->m[1] = 0.0f;
        motors_out->m[2] = 0.0f;
        motors_out->m[3] = 0.0f;
        return;
    }

    s_fusion.update(imu.gx, imu.gy, imu.gz,
                    imu.ax, imu.ay, imu.az,
                    SIM_DT);
    AttitudeDeg att = s_fusion.attitude();
    att_out->roll   = att.roll;
    att_out->pitch  = att.pitch;
    att_out->yaw    = att.yaw;

    float pos_y = in->obs[11];
    float tgt_y = in->obs[13];
    float vel_y = in->obs[8];
    float alt_err = tgt_y - pos_y;

    const float KP_ALT = 0.15f;   
    const float KD_ALT = 0.12f;
    const float THRUST_BASE  = 0.30f;   
    const float THRUST_MIN   = 0.05f;
    const float THRUST_MAX   = 0.80f;   

    float thrust = THRUST_BASE
                 + (KP_ALT * alt_err)
                 - (KD_ALT * vel_y);
    if (thrust > THRUST_MAX) thrust = THRUST_MAX;
    if (thrust < THRUST_MIN) thrust = THRUST_MIN;

    motors_out->m[0] = thrust;
    motors_out->m[1] = (float)base_thrust;   
    motors_out->m[2] = 0.0f;
    motors_out->m[3] = 0.0f;
}