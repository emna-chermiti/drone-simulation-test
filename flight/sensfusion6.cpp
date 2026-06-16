#include "sensfusion6.h"
#include <Arduino.h>
#include <math.h>

void SensFusion6::init(float rollDeg, float pitchDeg, float yawDeg) {
    _att.roll = rollDeg;
    _att.pitch = pitchDeg;
    _att.yaw = yawDeg;
}
void SensFusion6::resetYaw() {
    _att.yaw = 0.0f;
}
void SensFusion6::update(float gxDps, float gyDps, float gzDps,
                         float axG, float ayG, float azG, float dt) {
    
    const float alpha = 0.98f; 
    
    float accelRoll = atan2f(ayG, sqrtf(axG * axG + azG * azG) + 0.001f) * 180.0f / PI;
    float accelPitch = atan2f(-axG, sqrtf(ayG * ayG + azG * azG) + 0.001f) * 180.0f / PI;

    _att.roll = alpha * (_att.roll + gxDps * dt) + (1.0f - alpha) * accelRoll;
    _att.pitch = alpha * (_att.pitch + gyDps * dt) + (1.0f - alpha) * accelPitch;
    
    _att.yaw += gzDps * dt;

    if (_att.yaw > 180.0f)  _att.yaw -= 360.0f;
    if (_att.yaw < -180.0f) _att.yaw += 360.0f;
}

AttitudeDeg SensFusion6::attitude() const {
    return _att;
}
