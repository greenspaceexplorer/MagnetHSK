#ifndef LHELEVEL_H
#define LHELEVEL_H

#include <MagnetHSK_protocol.h>
#include <Arduino.h>
#include <Wire.h>

class LHeLevel
{

public:
    LHeLevel(TwoWire &dacI2Cport, uint8_t nearADCpin, uint8_t nearIMONpin, uint8_t farADCpin, uint8_t farIMONpin, uint8_t CLRpin);
    ~LHeLevel();

    void setup();
    sHeliumLevel read(uint8_t id);
    sHeliumLevels read();
    bool apply_current(uint8_t id, float current);
    uint16_t level_adc(uint8_t id);
    uint16_t current_adc(uint8_t id);
    float monitor_current(uint8_t id);
    void clearDAC();
    
    uint16_t current_to_dac(float current);

private:
    uint8_t nearADC, nearIMON, farADC, farIMON, CLR;
    TwoWire *dacI2C;
    sHeliumLevel near, far;
    uint8_t cmd[3];
    const uint16_t error = 65535;
    const uint8_t addr = 0b00001111;
    uint16_t iread, ideice, imon, vmon;
    const uint8_t DAC_bits = 12;
    float Rlim = 6620, Vlim = 0.75, Vrefout = 2.5, Rdiv1 = 5620, Rdiv2 = 1000;
    float dac_div = Rdiv2/(Rdiv1+Rdiv2);
    float i2adc = pow(2,DAC_bits)*Vlim*Rlim/(dac_div*2*Vrefout*1000*0.8);
};
#endif