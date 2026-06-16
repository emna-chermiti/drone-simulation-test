#include "pid_advanced.h"

void PidAdvanced::init(float kp, float ki, float kd, float dt) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _dt = dt;
    reset();
}

void PidAdvanced::setDt(float dt)               { _dt = dt; }
void PidAdvanced::setDesired(float desired)     { _desired = desired; }
void PidAdvanced::setIntegralLimit(float limit) { _integralLimit = limit; }
void PidAdvanced::setOutputLimit(float limit)   { _outputLimit = limit; }


void PidAdvanced::reset() {
    _integral  = 0.0f;
    _prevError = 0.0f;
}

float PidAdvanced::update(float measured) {
    float error = _desired - measured;

    float p = _kp * error;

    _integral += error * _dt;
    _integral  = constrain(_integral, -_integralLimit, _integralLimit);
    float i    = _ki * _integral;

    float derivative = (error - _prevError) / _dt;
    float d          = _kd * derivative;
    _prevError       = error;

    float output = p + i + d;
    return constrain(output, -_outputLimit, _outputLimit);
}

float PidAdvanced::updateWithRate(float measured, float measuredRate) {
    float error = _desired - measured;

    float p = _kp * error;

    _integral += error * _dt;
    _integral  = constrain(_integral, -_integralLimit, _integralLimit);
    float i    = _ki * _integral;

    float d = -_kd * measuredRate;

    float output = p + i + d;
    return constrain(output, -_outputLimit, _outputLimit);
}