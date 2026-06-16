#include "stabilizer.h"

void Stabilizer::init(FlightSystem& fs) {
    _fs = &fs;

    _fusion.init(0.0f, 0.0f, 0.0f);

    _pid.init(0.002f);
    _pid.reset(); 

    _lastUs = micros();

    Serial.println("[STAB] Stabilizer initialized.");
}

MotorMix Stabilizer::update(const Setpoint& setpoint, const IMUData& imu) {

    unsigned long nowUs = micros();
    _dt = (nowUs - _lastUs) / 1000000.0f;
    _lastUs = nowUs;
    if (_dt < 0.0005f || _dt > 0.05f) {
        _dt = 0.002f; 
    }
    _fusion.update(imu.gx, imu.gy, imu.gz,
                   imu.ax, imu.ay, imu.az,
                   _dt);
    AttitudeDeg att = _fusion.attitude();

    if (_fs->shouldCutPower()) {
        MotorMix zero = {0, 0, 0, 0};
        return zero;
    }

    if (!imu.isHealthy) {
        return emergencyStop("IMU unhealthy in stabilizer");
    }

    AttitudeRateDps rate;
    rate.roll  = imu.gx; 
    rate.pitch = imu.gy; 
    rate.yaw   = imu.gz; 

    _pid.setDt(_dt);
    ControlOutput correction = _pid.update(setpoint, att, rate);
    MotorMix mix = _mixer.mixX(HOVER_THRUST, correction);

    return mix;
}

MotorMix Stabilizer::emergencyStop(const char* reason) {
    stopAllMotors(); 

    if (_fs) _fs->triggerEmergencyStop(reason);

    _pid.reset();

    MotorMix zero = {0, 0, 0, 0};
    return zero;
}

AttitudeDeg Stabilizer::getAttitude() const {
    return _fusion.attitude();
}