#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

#include "flight_types.h"
#include "pid_advanced.h"


class ControllerPid {
public:
    void init(float dt);           
    void setDt(float dt);          
    void reset();                  

    ControlOutput update(const Setpoint&        setpoint,
                         const AttitudeDeg&     attitude,
                         const AttitudeRateDps& rate);

private:
    PidAdvanced _attRoll;  
    PidAdvanced _attPitch;  
    PidAdvanced _yawRate;  

    float _dt = 0.002f; 
};

#endif 