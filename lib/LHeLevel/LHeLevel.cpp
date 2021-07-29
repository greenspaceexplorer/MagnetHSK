#include "LHeLevel.h"

LHeLevel::LHeLevel(TwoWire &dacI2Cport, uint8_t nearADCpin, uint8_t nearIMONpin, uint8_t farADCpin, uint8_t farIMONpin, uint8_t CLRpin)
{
    // store I2C port and ADC pins
    dacI2C = &dacI2Cport;
    nearADC = nearADCpin;
    nearIMON = nearIMONpin;
    farADC = farADCpin;
    farIMON = farIMONpin;
    CLR = CLRpin;

    // default level probe values
    near.level = error;
    far.level = error;
}

LHeLevel::~LHeLevel()
{
}

sHeliumLevels LHeLevel::read()
{
    near.level = error;
    far.level = error;
    sHeliumLevels out;
    out.near = near;
    out.far  = far;
    return out;
}

uint16_t LHeLevel::current_adc(uint8_t id)
{
    switch (id)
    {
    case eHeliumLevelNear:
        return analogRead(nearIMON);
        break;
    case eHeliumLevelFar:
        return analogRead(farIMON);
        break;
    default:
        break;
    }
    return error;
}
uint16_t LHeLevel::level_adc(uint8_t id)
{
    switch (id)
    {
    case eHeliumLevelNear:
        return analogRead(nearADC);
        break;
    case eHeliumLevelFar:
        return analogRead(farADC);
        break;
    default:
        break;
    }
    return error;
}

void LHeLevel::setup()
{
    // begin I2C communication
    dacI2C->begin();

    // set ADC pins to high impedance for readout
    pinMode(nearADC,INPUT);
    pinMode(nearIMON,INPUT);
    pinMode(farADC,INPUT);
    pinMode(farIMON,INPUT);
    
    // clear pin
    pinMode(CLR,OUTPUT);
    digitalWrite(CLR,HIGH);
    

    // DEBUG: test pin
    pinMode(PB_6,OUTPUT);
    digitalWrite(PB_6,HIGH);
}

float LHeLevel::monitor_current(uint8_t id)
{
    uint16_t imon_adc = current_adc(id);
    float out;
    out = 3.3*float(imon_adc)/4095.0;
    out = 1000*out/Rlim;
    return out;

}

bool LHeLevel::apply_current(uint8_t id, float current)
{
    uint8_t address = addr;
    uint8_t cmd_byte;
    uint16_t dacData = current_to_dac(current);
    switch (id)
    {
    case eHeliumLevelNear:
        // address <<= 1;
        // address |= 0; // address byte: write
        cmd_byte = 0b00000000;
        memcpy(&cmd[0],(uint8_t *)&cmd_byte,sizeof(uint8_t));
        memcpy(&cmd[1],(uint16_t *)&dacData,sizeof(uint16_t));

        dacI2C->beginTransmission(address);
        dacI2C->write(cmd,3);
        dacI2C->endTransmission();
        Serial.println(address);
        Serial.print(cmd[0],BIN);
        Serial.print("|");
        Serial.print(cmd[1],BIN);
        Serial.print("|");
        Serial.println(cmd[2],BIN);
        Serial.println(dacData);

        return 1;
        break;
    case eHeliumLevelFar:
        // address <<= 1;
        // address |= 0; // address byte: write
        cmd_byte = 0b00000001;
        memcpy(&cmd[0],(uint8_t *)&cmd_byte,sizeof(uint8_t));
        memcpy(&cmd[1],(uint16_t *)&dacData,sizeof(uint16_t));

        dacI2C->beginTransmission(address);
        dacI2C->write(cmd,3);
        dacI2C->endTransmission();
        Serial.println(address);
        Serial.print(cmd[0],BIN);
        Serial.print("|");
        Serial.print(cmd[1],BIN);
        Serial.print("|");
        Serial.println(cmd[2],BIN);
        Serial.println(dacData);

        return 1;
        break;
    case eHeliumLevels:
        // address <<= 1;
        // address |= 0; // address byte: write
        cmd_byte = 0b00011111;
        // memcpy(&cmd[0],(uint8_t *)&cmd_byte,sizeof(uint8_t));
        // memcpy(&cmd[1],(uint16_t *)&dacData,sizeof(uint16_t));

        // DEBUG
        digitalWrite(PB_6,LOW); 
        dacI2C->beginTransmission(address);
        // dacI2C->write(cmd,3);
        dacI2C->write(cmd_byte);
        dacI2C->write(dacData >> 8);
        dacI2C->write(dacData);
        dacI2C->endTransmission();
        // DEBUG
        digitalWrite(PB_6,HIGH); 
        Serial.println(address);
        Serial.print(cmd[0],BIN);
        Serial.print("|");
        Serial.print(cmd[1],BIN);
        Serial.print("|");
        Serial.println(cmd[2],BIN);
        Serial.println(dacData);
        Serial.println(dacData>>8);
        Serial.println(uint8_t(dacData));

        return 1;

        break;
    default:
        break;
    }
    return 0;
    
}

sHeliumLevel LHeLevel::read(uint8_t id)
{
    switch (id)
    {
    case eHeliumLevelNear:
        near.level = error;
        near.level -= 1;
        return near;
        break;
    case eHeliumLevelFar:
        far.level = error;
        far.level -= 1;
        return far;
        break;
    default:
        near.level = error;
        near.level -= 1;
        break;
    }
    return near;
}

uint16_t LHeLevel::current_to_dac(float current)
{
    current /= 1000.0;

    float ilim = 800.0/Rlim;
    if(current < 0) current*=-1;
    if(current > ilim) current = ilim;
    float out_fl = i2adc*current;
    uint16_t out = uint16_t(out_fl);
    out = out << 4;
    return out; 
}