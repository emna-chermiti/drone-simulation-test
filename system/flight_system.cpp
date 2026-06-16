#include "flight_system.h"
#include "motors.h"

#define MAX_TILT_DEG  55.0f

void FlightSystem::init(unsigned long armDelayMs, unsigned long hoverDurationMs) {
    _armDelayMs      = armDelayMs;
    _hoverDurationMs = hoverDurationMs;
    _emergencyStop   = false;
    _stopReason      = nullptr;
    _enterPhase(FlightPhase::BOOT);
    Serial.println("[FSM] FlightSystem initialized — phase: BOOT");
}

void FlightSystem::_enterPhase(FlightPhase next) {
    _phase        = next;
    _phaseStartMs = millis();
    Serial.printf("[FSM] → Phase: %s  (t=%lums)\n", getPhaseName(), _phaseStartMs);
}

void FlightSystem::notifyCalibrationDone() {
    if (_phase != FlightPhase::BOOT) return; 
    _enterPhase(FlightPhase::READY);
    Serial.printf("[FSM] Arming in %.1f seconds — keep drone flat.\n",
                  _armDelayMs / 1000.0f);
}

void FlightSystem::notifyArmingComplete() {
    if (_phase != FlightPhase::ARMING) return; 
    _enterPhase(FlightPhase::HOVERING);
    Serial.printf("[FSM] Hovering for %.1f seconds then auto-disarm.\n",
                  _hoverDurationMs / 1000.0f);
}

void FlightSystem::triggerEmergencyStop(const char* reason) {
    if (_phase == FlightPhase::HALTED) return; 
    _emergencyStop = true;
    _stopReason    = reason;
    stopAllMotors(); 
    _enterPhase(FlightPhase::HALTED);
    Serial.printf("[FSM] *** EMERGENCY STOP: %s ***\n", reason);
}

void FlightSystem::update(const SafetyInputs& s) {

    if (!s.imuHealthy && _phase == FlightPhase::HOVERING) {
        triggerEmergencyStop("IMU unhealthy");
        return;
    }

    if ((fabsf(s.rollDeg)  > MAX_TILT_DEG ||
         fabsf(s.pitchDeg) > MAX_TILT_DEG) &&
         _phase == FlightPhase::HOVERING) {
        triggerEmergencyStop("Tilt limit exceeded");
        return;
    }

    unsigned long elapsed = millis() - _phaseStartMs;

    switch (_phase) {

        case FlightPhase::BOOT:
            break;

        case FlightPhase::READY:
            if (elapsed >= _armDelayMs) {
                _enterPhase(FlightPhase::ARMING);
            }
            break;

        case FlightPhase::ARMING:
            break;

        case FlightPhase::HOVERING:
            if (elapsed >= _hoverDurationMs) {
                Serial.println("[FSM] Hover time complete — initiating disarm.");
                _enterPhase(FlightPhase::DISARMING);
                stopAllMotors(); 
            }
            break;

        case FlightPhase::DISARMING:
            if (elapsed >= 1000) {
                _enterPhase(FlightPhase::HALTED);
                Serial.println("[FSM] Mission complete. Safe to handle.");
            }
            break;

        case FlightPhase::HALTED:
            break;
    }
}
bool FlightSystem::shouldCutPower() const {
    return _emergencyStop || (_phase != FlightPhase::HOVERING);
}

bool FlightSystem::isHovering() const {
    return _phase == FlightPhase::HOVERING;
}

bool FlightSystem::isHalted() const {
    return _phase == FlightPhase::HALTED;
}

FlightPhase FlightSystem::getPhase() const {
    return _phase;
}

const char* FlightSystem::getPhaseName() const {
    switch (_phase) {
        case FlightPhase::BOOT:      return "BOOT";
        case FlightPhase::READY:     return "READY";
        case FlightPhase::ARMING:    return "ARMING";
        case FlightPhase::HOVERING:  return "HOVERING";
        case FlightPhase::DISARMING: return "DISARMING";
        case FlightPhase::HALTED:    return "HALTED";
        default:                     return "UNKNOWN";
    }
}