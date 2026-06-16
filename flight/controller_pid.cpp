#include "controller_pid.h"

#define ROLL_KP   2.0f
#define ROLL_KI   0.5f
#define ROLL_KD   0.05f

#define PITCH_KP  2.0f
#define PITCH_KI  0.5f
#define PITCH_KD  0.05f

#define YAW_KP    5.0f
#define YAW_KI    0.8f
#define YAW_KD    0.0f

void ControllerPid::init(float dt) {
    _dt = dt;

    _attRoll.init (ROLL_KP,  ROLL_KI,  ROLL_KD,  _dt);
    _attPitch.init(PITCH_KP, PITCH_KI, PITCH_KD, _dt);
    _yawRate.init (YAW_KP,   YAW_KI,   YAW_KD,   _dt);

    _attRoll.setIntegralLimit (15.0f);
    _attPitch.setIntegralLimit(15.0f);
    _yawRate.setIntegralLimit (10.0f);

    _attRoll.setOutputLimit (80.0f);
    _attPitch.setOutputLimit(80.0f);
    _yawRate.setOutputLimit (60.0f); 

    Serial.println("[PID] Controller initialized.");
}

void ControllerPid::setDt(float dt) {
    _dt = dt;
    _attRoll.setDt(dt);
    _attPitch.setDt(dt);
    _yawRate.setDt(dt);
}


void ControllerPid::reset() {
    _attRoll.reset();
    _attPitch.reset();
    _yawRate.reset();
    Serial.println("[PID] Integral state cleared.");
}

ControlOutput ControllerPid::update(const Setpoint&        setpoint,
                                     const AttitudeDeg&     attitude,
                                     const AttitudeRateDps& rate) {
    ControlOutput out;

    _attRoll.setDesired (setpoint.attitudeDeg.roll);
    _attPitch.setDesired(setpoint.attitudeDeg.pitch);
    _yawRate.setDesired (setpoint.attitudeRateDps.yaw); 

    out.roll  = _attRoll.updateWithRate (attitude.roll,  rate.roll);

    out.pitch = _attPitch.updateWithRate(attitude.pitch, rate.pitch);

    out.yaw   = _yawRate.updateWithRate (attitude.yaw,   rate.yaw);

    return out;
}