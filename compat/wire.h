// compat/Wire.h
// No-op Wire stub, sim bypasses I2C entirely
#pragma once
#include <cstdint>

struct TwoWire {
    void    begin(int sda, int scl)              { (void)sda; (void)scl; }
    void    setClock(uint32_t freq)              { (void)freq; }
    void    setTimeOut(uint32_t ms)              { (void)ms; }
    void    beginTransmission(uint8_t addr)      { (void)addr; }
    uint8_t endTransmission(bool stop = true)    { (void)stop; return 0; }
    uint8_t requestFrom(uint8_t addr, uint8_t len,
                        uint8_t stop = 1) {
        (void)addr; (void)stop;
        _avail = len;
        return len;
    }
    void    write(uint8_t b)  { (void)b; }
    uint8_t read()            { return 0; }
    int     available()       { return _avail; }
private:
    int _avail = 0;
};
extern TwoWire Wire;