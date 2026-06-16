#include "imu.h"
#include "compat/i2c_bus.h"
#include <math.h>

#define MPU_ADDR         0x68  // I2C address AD0 pin tied LOW on GY-521 module
#define REG_PWR_MGMT_1   0x6B  // Power management: sleep bit + clock source
#define REG_SMPLRT_DIV   0x19  // Sample rate divider
#define REG_CONFIG       0x1A  // DLPF configuration
#define REG_GYRO_CONFIG  0x1B  // Gyro full-scale range
#define REG_ACCEL_CONFIG 0x1C  // Accel full-scale range
#define REG_ACCEL_XOUT_H 0x3B  // Burst read start: accel[6] + temp[2] + gyro[6]
// ±8g    → LSB/g    = 4096   
// ±500°/s → LSB/dps = 65.5  
#define ACCEL_SCALE  4096.0f
#define GYRO_SCALE   65.5f

// 50ms = 20Hz minimum 
#define MAX_SAFE_DT  0.05f

// I2C Pins (ESP32-S3 Zero)
#define I2C_SDA_PIN  3
#define I2C_SCL_PIN  4
#define I2C_FREQ_HZ  400000  // Fast Mode 

static float gyroXoffset  = 0.0f;
static float gyroYoffset  = 0.0f;
static float gyroZoffset  = 0.0f;
static float accelXoffset = 0.0f;
static float accelYoffset = 0.0f;
static float accelZoffset = 0.0f;

static unsigned long lastTimeUs = 0;

void initIMU() {
    i2c_bus::init(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    delay(150); // MPU-6050 needs up to 100ms after power-on before accepting I2C
    i2c_bus::scan();

    // Default state after power-on: sleep mode, internal 8MHz RC oscillator
    // 0x01 = wake up + use PLL locked to X-axis gyro oscillator (more stable)
    if (!i2c_bus::writeByte(MPU_ADDR, REG_PWR_MGMT_1, 0x01)) {
        Serial.println("[IMU] FATAL: Cannot reach MPU-6050 — check wiring");
        while (true) delay(100); 
    }
    delay(10); // Allow PLL to lock

    i2c_bus::writeByte(MPU_ADDR, REG_CONFIG, 0x03);
    i2c_bus::writeByte(MPU_ADDR, REG_SMPLRT_DIV, 0x01);
    i2c_bus::writeByte(MPU_ADDR, REG_ACCEL_CONFIG, 0x10);
    i2c_bus::writeByte(MPU_ADDR, REG_GYRO_CONFIG, 0x08);

    Serial.println("[IMU] MPU-6050 configured.");

    calibrateIMU();

    lastTimeUs = micros(); // Seed the dt timer after calibration completes
}

void calibrateIMU() {
    Serial.println("[IMU] Calibrating — keep drone flat and still...");

    const int SAMPLES = 500;
    long sumAX = 0, sumAY = 0, sumAZ = 0;
    long sumGX = 0, sumGY = 0, sumGZ = 0;

    uint8_t buf[14];

    for (int i = 0; i < SAMPLES; i++) {
        if (!i2c_bus::readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
            Serial.printf("[IMU] Calibration read failed at sample %d\n", i);
            i--; 
            delay(5);
            continue;
        }

        sumAX += (int16_t)(buf[0]  << 8 | buf[1]);
        sumAY += (int16_t)(buf[2]  << 8 | buf[3]);
        sumAZ += (int16_t)(buf[4]  << 8 | buf[5]);
        sumGX += (int16_t)(buf[8]  << 8 | buf[9]);
        sumGY += (int16_t)(buf[10] << 8 | buf[11]);
        sumGZ += (int16_t)(buf[12] << 8 | buf[13]);

        delay(2); 
    }

    gyroXoffset = (float)sumGX / SAMPLES;
    gyroYoffset = (float)sumGY / SAMPLES;
    gyroZoffset = (float)sumGZ / SAMPLES;

    accelXoffset = (float)sumAX / SAMPLES;           
    accelYoffset = (float)sumAY / SAMPLES;          
    accelZoffset = ((float)sumAZ / SAMPLES) - ACCEL_SCALE; 

    Serial.println("[IMU] Calibration complete.");
    Serial.printf("[IMU] Gyro offsets  → X:%.2f  Y:%.2f  Z:%.2f (LSB)\n",
                  gyroXoffset, gyroYoffset, gyroZoffset);
    Serial.printf("[IMU] Accel offsets → X:%.2f  Y:%.2f  Z:%.2f (LSB)\n",
                  accelXoffset, accelYoffset, accelZoffset);
}


// scales to physical units, validates dt, and returns an IMUData struct.
// Returns isHealthy = false if:
//   - I2C read fails (after retry)
//   - dt is outside the valid range (stall or first-frame artifact)
//   - Accel magnitude is implausible (sensor fault or free-fall)
IMUData readIMU() {
    IMUData data = {};
    data.isHealthy = false;
    uint8_t buf[14];
    if (!i2c_bus::readBytes(MPU_ADDR, REG_ACCEL_XOUT_H, buf, 14)) {
        Serial.println("[IMU] ERROR: I2C read failed — flagging unhealthy");
        return data; // isHealthy = false
    }

    int16_t rAX = (int16_t)(buf[0]  << 8 | buf[1]);
    int16_t rAY = (int16_t)(buf[2]  << 8 | buf[3]);
    int16_t rAZ = (int16_t)(buf[4]  << 8 | buf[5]);
    int16_t rGX = (int16_t)(buf[8]  << 8 | buf[9]);
    int16_t rGY = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t rGZ = (int16_t)(buf[12] << 8 | buf[13]);
    unsigned long nowUs = micros();
    float dt = (nowUs - lastTimeUs) / 1000000.0f;
    lastTimeUs = nowUs;

    if (dt <= 0.0f || dt > MAX_SAFE_DT) {
        Serial.printf("[IMU] WARNING: Bad dt=%.4fs — skipping frame\n", dt);
        return data; // isHealthy = false
    }

    // Subtract the bias measured during calibration, then divide by scale factor
    data.ax = (rAX - accelXoffset) / ACCEL_SCALE; // G
    data.ay = (rAY - accelYoffset) / ACCEL_SCALE; // G
    data.az = (rAZ - accelZoffset) / ACCEL_SCALE; // G
    data.gx = (rGX - gyroXoffset)  / GYRO_SCALE;  // deg/s
    data.gy = (rGY - gyroYoffset)  / GYRO_SCALE;  // deg/s
    data.gz = (rGZ - gyroZoffset)  / GYRO_SCALE;  // deg/s

    //   < 0.3G → free-fall or sensor disconnected
    //   > 3.0G → severe impact or sensor fault
    // In both cases, the data is untrustworthy — flag unhealthy.
    float mag = sqrtf(data.ax*data.ax + data.ay*data.ay + data.az*data.az);
    if (mag < 0.3f || mag > 3.0f) {
        Serial.printf("[IMU] WARNING: Accel magnitude implausible: %.2fG\n", mag);
        return data; // isHealthy = false
    }

    //if all checks passed 
    data.isHealthy = true;
    return data;
}