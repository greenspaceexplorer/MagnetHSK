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

    void setup(uint32_t period);
    void update();
    void update(uint32_t period);
    void read_cycle();
    void set_probe(uint8_t probe);
    bool toggle_deice();

    sHeliumLevel read(uint8_t id);
    sHeliumLevels read();

    float apply_current(uint8_t id, double current);
    float apply_current(uint8_t id, uint16_t dacData);
    uint16_t level_adc(uint8_t id);
    uint16_t current_adc(uint8_t id);
    float monitor_current(uint8_t id);
    float lhe_level(uint16_t adc_read,uint16_t imon_read);
    void set_level(float level,uint8_t error, uint8_t id);
    void clearDAC();
    
    uint16_t current_to_dac(float current);
    float dac_to_current(uint16_t dac);

private:
    TwoWire *dacI2C;
    sHeliumLevel sError;
    sHeliumLevels levels;
    uint8_t nearADC, nearIMON, farADC, farIMON, CLR;
    uint16_t error = 65535;
    uint32_t last_read_near, last_read_far,read_period,read_timer;
    const uint8_t addr = 0b00001111;
    bool bdeice;
    float iread = 0.02, ideice = 0.1;
    
    uint8_t cycle_probe;
    uint16_t tread = 6000, tdeice = 2000, cycle_start, cycle_timer, cycle_period;
    bool cycle_busy;
    float near_cycle_level, far_cycle_level;


    const uint8_t DAC_bits = 12;
    float Rlim = 6650, Vlim = 0.75, Vrefout = 2.5, Rdiv1 = 5620, Rdiv2 = 1000;
    float dac_div = Rdiv2/(Rdiv1+Rdiv2);
    float i2adc = pow(2,DAC_bits)*Vlim*Rlim/(dac_div*2*Vrefout*1000*0.8);
};
#endif