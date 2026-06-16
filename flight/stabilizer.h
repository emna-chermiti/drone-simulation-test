#ifndef STABILIZER_H
#define STABILIZER_H

#include "imu.h"
#include "sensfusion6.h"
#include "controller_pid.h"
#include "power_distrib.h"
#include "flight_system.h"
#include "motors.h"


class Stabilizer {
public:

    void init(FlightSystem& fs);
    MotorMix update(const Setpoint& setpoint, const IMUData& imu);
    MotorMix emergencyStop(const char* reason);
    AttitudeDeg getAttitude() const;

private:
    SensFusion6      _fusion;       
    ControllerPid    _pid;          
    PowerDistribution _mixer;       
    FlightSystem*    _fs = nullptr; 

    unsigned long _lastUs  = 0;     
    float         _dt      = 0.002f;

    static constexpr uint8_t HOVER_THRUST = 120;
};

#endif 
