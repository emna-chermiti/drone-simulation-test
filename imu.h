#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// Raw calibrated sensor output 

struct IMUData {
    float ax, ay, az;  // Accelerometer in G  
    float gx, gy, gz;  // Gyroscope in deg/s  
    bool  isHealthy;   // false → stabilizer must trigger emergency stop
};

void initIMU();      // Init I2C via i2c_bus, configure MPU-6050, run calibration
void calibrateIMU(); // Collect bias offsets 
IMUData readIMU();   // Read 14 bytes, apply offsets, return calibrated values

#endif 