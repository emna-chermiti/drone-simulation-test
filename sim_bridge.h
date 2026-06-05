// sim_bridge.h
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// Raw input from Python each tick
typedef struct {
    float obs[16];       // full PyFlyt obs array
    float accel_ms2[3];  // noisy accel from imu_simulator.py in m/s²
} SimInput;

// Fused attitude — output of your SensFusion6
typedef struct {
    float roll, pitch, yaw;  // degrees
} SimAttitude;

// Final motor output — normalized for PyFlyt
typedef struct {
    float m[4];  // 0.0 to 1.0 per motor
} SimMotors;

void sim_init(void);
void sim_reset(void);

// Phase 2: sensor fusion only
void sim_update_fusion(const SimInput* in, SimAttitude* att_out);

// Phase 3+: full pipeline
void sim_update_full(const SimInput* in,
                     float target_roll,
                     float target_pitch,
                     float target_yaw_rate,
                     uint8_t base_thrust,
                     SimMotors* motors_out,
                     SimAttitude* att_out);

#ifdef __cplusplus
}
#endif