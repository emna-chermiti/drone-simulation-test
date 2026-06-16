#ifndef PID_ADVANCED_H
#define PID_ADVANCED_H

#include <Arduino.h>


class PidAdvanced {
public:
    void init(float kp, float ki, float kd, float dt);

    void setDt(float dt);
    void setDesired(float desired);
    void setIntegralLimit(float limit);
    void setOutputLimit(float limit);

    float update(float measured);

    float updateWithRate(float measured, float measuredRate);

    void reset();

private:
    float _kp = 0.0f;
    float _ki = 0.0f;
    float _kd = 0.0f;
    float _dt = 0.004f;

    float _desired       = 0.0f;
    float _integral      = 0.0f;
    float _prevError     = 0.0f;
    float _integralLimit = 50.0f;  
    float _outputLimit   = 200.0f; 
};

#endif 