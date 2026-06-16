#ifndef FLIGHT_SYSTEM_H
#define FLIGHT_SYSTEM_H

#include <Arduino.h>

enum class FlightPhase {
    BOOT,           
    READY,          
    ARMING,         
    HOVERING,       
    DISARMING,      
    HALTED          
};

class FlightSystem {
public:
    void init(unsigned long armDelayMs, unsigned long hoverDurationMs);
    void update(const struct SafetyInputs& inputs); 
    void triggerEmergencyStop(const char* reason);  

    bool shouldCutPower()  const;
    bool isHovering()      const;
    bool isHalted()        const; 
    FlightPhase getPhase() const;
    const char* getPhaseName() const;

    void notifyCalibrationDone(); 
    void notifyArmingComplete();  

private:
    FlightPhase _phase = FlightPhase::BOOT;

    unsigned long _armDelayMs      = 3000; 
    unsigned long _hoverDurationMs = 15000;
    unsigned long _phaseStartMs    = 0;   

    bool _emergencyStop = false;
    const char* _stopReason = nullptr;

    void _enterPhase(FlightPhase next);
};

struct SafetyInputs {
    float rollDeg;   
    float pitchDeg;   
    bool  imuHealthy; 
};

#endif 