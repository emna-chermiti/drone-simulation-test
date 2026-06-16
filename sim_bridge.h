#pragma once
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float obs[16];       
    float accel_ms2[3]; 
} SimInput;

typedef struct {
    float roll, pitch, yaw;  // degrees
} SimAttitude;

typedef struct {
    float m[4];  
} SimMotors;

void sim_init(void);
void sim_reset(void);

void sim_update_fusion(const SimInput* in, SimAttitude* att_out);


void sim_update_full(const SimInput* in,
                     float target_roll,
                     float target_pitch,
                     float target_yaw_rate,
                     uint8_t  base_thrust,
                     SimMotors* motors_out,
                     SimAttitude* att_out);


void sim_update_highlevel(const SimInput* in,
                          float target_yaw_rate,
                          float roll_cmd,
                          float pitch_cmd,
                          uint8_t base_thrust,
                          SimMotors* motors_out,
                          SimAttitude* att_out);

#ifdef __cplusplus
}
#endif