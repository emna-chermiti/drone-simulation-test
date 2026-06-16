#include "../compat/Arduino.h" 
#include "power_distrib.h"
#include <algorithm>

// PyFlyt Native Channel Layout array map:
// Array index [0] -> Front-Right (FR)
// Array index [1] -> Rear-Left   (RL)
// Array index [2] -> Front-Left  (FL)
// Array index [3] -> Rear-Right  (RR)

MotorMix PowerDistribution::mixX(uint8_t thrust, const ControlOutput& c) const {
    MotorMix mix;

    if (thrust == 0) {
        mix.m1 = mix.m2 = mix.m3 = mix.m4 = 0.0f;
        return mix;
    }

    float m1 = (float)thrust - c.roll + c.pitch + c.yaw;  
    float m2 = (float)thrust + c.roll - c.pitch + c.yaw;  
    float m3 = (float)thrust + c.roll + c.pitch - c.yaw;  
    float m4 = (float)thrust - c.roll - c.pitch - c.yaw;  

    float maxOut = std::max({m1, m2, m3, m4});
    if (maxOut > MOTOR_MAX) {
        float scale = (float)MOTOR_MAX / maxOut;
        m1 *= scale;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
    }

    auto applyFloor = [](float val) -> float {
        if (val < (float)MOTOR_MIN) val = (float)MOTOR_MIN;
        if (val > (float)MOTOR_MAX) val = (float)MOTOR_MAX; 
        return val;
    };
    mix.m1 = applyFloor(m1);
    mix.m2 = applyFloor(m2);
    mix.m3 = applyFloor(m3);
    mix.m4 = applyFloor(m4);

    return mix;
}