#ifndef TEMPERATUREPROBE_H
#define TEMPERATUREPROBE_H

#include <MagnetHSK_protocol.h>
#include <Arduino.h>
#include <OneWire.h>
float tempSensorVal(int tempNum, int pinNum);

class TemperatureProbe{

    public:
        TemperatureProbe();
        ~TemperatureProbe();
        sTempProbeAll readAll();
        sTempProbe read(uint8_t cmd);
        sTempProbe read(uint64_t address);
        
        sTempProbe noprobe;
        
    private:
        int16_t noerror = 32767;
        uint64_t badAddress = 0XFFFFFFFFFFFFFFFF;

};


#endif