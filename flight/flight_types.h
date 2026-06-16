#ifndef FLIGHT_TYPES_H
#define FLIGHT_TYPES_H

struct AttitudeDeg {
    float roll  = 0.0f;  // + = right side down
    float pitch = 0.0f;  // + = nose down
    float yaw   = 0.0f;  // + = rotate CCW (gyro-integrated, drifts)
};

struct AttitudeRateDps {
    float roll  = 0.0f;  // gx from IMU
    float pitch = 0.0f;  // gy from IMU
    float yaw   = 0.0f;  // gz from IMU
};

struct Setpoint {
    AttitudeDeg     attitudeDeg;     // Target Euler angles
    AttitudeRateDps attitudeRateDps; // Target angular rates (yaw uses rate control)
};


struct ControlOutput {
    float roll  = 0.0f; 
    float pitch = 0.0f;  
    float yaw   = 0.0f;  
};

#endif // FLIGHT_TYPES_H