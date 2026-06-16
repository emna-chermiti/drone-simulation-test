#ifndef POWER_DISTRIB_H
#define POWER_DISTRIB_H

#include <Arduino.h>
#include "flight/flight_types.h"
#include "motors.h" 

struct MotorMix {
    float m1; // Front-Left  (CCW)
    float m2; // Front-Right (CW)
    float m3; // Rear-Right  (CCW)
    float m4; // Rear-Left   (CW)
};


class PowerDistribution {
public:
    MotorMix mixX(uint8_t thrust, const ControlOutput& c) const;
};

#endif 