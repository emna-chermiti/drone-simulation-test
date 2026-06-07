#include "../compat/Arduino.h" 
#include "power_distrib.h"

// ─── Quad-X Motor Layout (top-down view) ──────────────────────────────────────
//
//       FRONT
//   M1(CCW)  M2(CW)
//       \      /
//        [BODY]
//       /      \
//   M4(CW)  M3(CCW)
//       REAR
//
// CCW motors: M1 (Front-Left), M3 (Rear-Right)
// CW  motors: M2 (Front-Right), M4 (Rear-Left)
//
// Mixing sign table (PyFlyt Simulation Corrected):
// ─────────────────────────────────────────────────
//        │ Thrust │ Roll │ Pitch │ Yaw
// ───────┼────────┼──────┼───────┼─────
//   M1   │   +    │  -   │   -   │  +   (CCW → yaw +)
//   M2   │   +    │  +   │   -   │  -   (CW  → yaw -)
//   M3   │   +    │  +   │   +   │  +   (CCW → yaw +)
//   M4   │   +    │  -   │   +   │  -   (CW  → yaw -)
// ─────────────────────────────────────────────────

MotorMix PowerDistribution::mixX(uint8_t thrust, const ControlOutput& c) const {
    MotorMix mix;

    // Hard zero when thrust is 0 (disarmed / pre-flight) ───────────────────
    if (thrust == 0) {
        mix.m1 = mix.m2 = mix.m3 = mix.m4 = 0;
        return mix;
    }

    // ── Apply PyFlyt-Corrected Quad-X Mixing ──────────────────────────────────
    // All values are in float to preserve fractional corrections before rounding.
    // Fixed: Signs are inverted for Roll and Pitch to counteract the double-inversion bug.
    float m1 = (float)thrust - c.roll - c.pitch + c.yaw;  // Front-Left  CCW
    float m2 = (float)thrust + c.roll - c.pitch - c.yaw;  // Front-Right CW
    float m3 = (float)thrust + c.roll + c.pitch + c.yaw;  // Rear-Right  CCW
    float m4 = (float)thrust - c.roll + c.pitch - c.yaw;  // Rear-Left   CW

    // ── Priority Scaling (preserve mix ratios instead of hard-clamping) ───────
    float maxOut = max({m1, m2, m3, m4});
    if (maxOut > MOTOR_MAX) {
        float scale = (float)MOTOR_MAX / maxOut;
        m1 *= scale;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
    }

    // ── Apply Idle Floor (during active flight only) ──────────────────────────
    auto applyFloor = [](float val) -> uint8_t {
        if (val < MOTOR_MIN) val = MOTOR_MIN;
        if (val > MOTOR_MAX) val = MOTOR_MAX; // Final safety ceiling
        return (uint8_t)val;
    };

    mix.m1 = applyFloor(m1);
    mix.m2 = applyFloor(m2);
    mix.m3 = applyFloor(m3);
    mix.m4 = applyFloor(m4);

    return mix;
}