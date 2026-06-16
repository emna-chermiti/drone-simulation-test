#ifndef SENSFUSION6_H
#define SENSFUSION6_H

#include "flight_types.h"


class SensFusion6 {
public:
    void init(float rollDeg, float pitchDeg, float yawDeg);
    void update(float gxDps, float gyDps, float gzDps,
                float axG,   float ayG,   float azG,
                float dt);

    AttitudeDeg attitude() const;

    void resetYaw();

private:
    AttitudeDeg _att;
    static constexpr float ALPHA = 0.98f;
    static constexpr float ACCEL_MIN_G = 0.7f;  
    static constexpr float ACCEL_MAX_G = 1.3f;  
};

#endif 