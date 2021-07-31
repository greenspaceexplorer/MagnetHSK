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

    // 2^32-1 as default last read
    last_read_near = 4294967295;
    last_read_far = 4294967295;

    // default level probe values
    levels.near.level = -1.0;
    levels.far.level = -1.0;
    levels.near.time_since_read =  last_read_near;
    levels.far.time_since_read = last_read_far;
    levels.near.error = 0;
    levels.far.error = 0;
    
    // error level
    sError.error = 0;
    sError.level = -1.0;
    sError.time_since_read = last_read_near;
    
    // default is no deicing
    bdeice = false;

    // initialize timers
    read_timer = 0;
    cycle_timer= 0;
    cycle_busy = false;
    cycle_period = bdeice ? tread + tdeice : tread;
    
    //set probe to read each cycle
    cycle_probe = eHeliumLevels;
    
}

LHeLevel::~LHeLevel()
{
}

void LHeLevel::setup(uint32_t period)
{
    // set the read period
    read_period = period;

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
    

    // turn on internal voltage reference
    uint8_t address = addr;
    uint8_t cmd_byte = 0b00111111;
    uint16_t dacData = 1;
    dacI2C->beginTransmission(address);
    dacI2C->write(cmd_byte);
    dacI2C->write(dacData >> 8);
    dacI2C->write(dacData);
    dacI2C->endTransmission();
    
    Serial.println("Probe setup...");
    // begin with a read cycle
    // delay(10);
    // read_cycle(); 

    // Serial.println("Probe setup cycle...");
}

sHeliumLevel LHeLevel::read(uint8_t id)
{
    switch (id)
    {
    case eHeliumLevelNear:
        levels.near.time_since_read = millis() - last_read_near;
        return levels.near;
        break;
    case eHeliumLevelFar:
        levels.far.time_since_read = millis() - last_read_far;
        return levels.far;
        break;
    default:
        break;
    }
    sError.error = 1;
    uint32_t recent_read = (last_read_near > last_read_far) ? last_read_near : last_read_far;
    sError.time_since_read = millis() - recent_read;
    return sError;
}

sHeliumLevels LHeLevel::read()
{
    levels.near.time_since_read = millis() - last_read_near;
    levels.far.time_since_read = millis() - last_read_far;
    return levels;
}

void LHeLevel::update()
{
    // millis()%read_period should be > read timer unless we've rolled over to a new period
    // if a read cycle is ongoing, continue calling read_cycle()
    if(millis()%read_period < read_timer || cycle_busy)
    {
        read_cycle();
        return;
    }
    read_timer = millis()%read_period;
    
}

void LHeLevel::read_cycle()
{
    delay(50);
    // start cycle if it was not previously busy 
    if(!cycle_busy)
    {
        Serial.println("Start cycle...");
        // start the cycle timer and busy signal
        cycle_start = millis();
        cycle_busy = true;
        if(bdeice)
        {
            apply_current(cycle_probe,ideice);

        }
        else
        {
            apply_current(cycle_probe,iread);
        }
        near_cycle_level = 0.0;
        far_cycle_level = 0.0;
        return;
    }
    
    // update the cycle timer
    cycle_timer = millis() - cycle_start;
    
    // continue based on where we are at with the timer
    
    // If we're deicing:
    // Continue deicing for tdeice milliseconds, then apply read current
    if(bdeice && cycle_timer < tdeice)
    {
        Serial.println("Deicing...");
        return;
    }
    else if( bdeice && cycle_timer > tdeice)
    {
        apply_current(cycle_probe,iread);
        return;
    }
    
    // read level adcs and continue averaging until the cycle period is over
    if(cycle_timer < cycle_period)
    {
        Serial.println("Reading...");
        switch (cycle_probe)
        {
        case eHeliumLevelNear:
            Serial.println("Reading near probe...");
            nearIMON = current_adc(eHeliumLevelNear);
            nearADC = level_adc(eHeliumLevelNear);
            if(near_cycle_level == 0.0) near_cycle_level = lhe_level(nearADC,nearIMON);
            near_cycle_level += lhe_level(nearADC,nearIMON);
            near_cycle_level /= 2.0;
            break;
        case eHeliumLevelFar:
            Serial.println("Reading far probe...");
            farIMON = current_adc(eHeliumLevelFar);
            farADC = level_adc(eHeliumLevelFar);
            if(far_cycle_level == 0.0) far_cycle_level = lhe_level(farADC,farIMON);
            far_cycle_level += lhe_level(farADC,farIMON);
            far_cycle_level /= 2.0;
            Serial.println(near_cycle_level);
            Serial.println(far_cycle_level);
            break;
        case eHeliumLevels:
            Serial.println("Reading both probes...");
            nearIMON = current_adc(eHeliumLevelNear);
            delay(1);
            nearADC = level_adc(eHeliumLevelNear);
            delay(1);
            if(near_cycle_level == 0.0) near_cycle_level = lhe_level(nearADC,nearIMON);
            near_cycle_level += lhe_level(nearADC,nearIMON);
            near_cycle_level /= 2.0;

            farIMON = current_adc(eHeliumLevelFar);
            delay(1);
            farADC = level_adc(eHeliumLevelFar);
            delay(1);
            if(far_cycle_level == 0.0) far_cycle_level = lhe_level(farADC,farIMON);
            far_cycle_level += lhe_level(farADC,farIMON);
            far_cycle_level /= 2.0;
            Serial.println(near_cycle_level);
            Serial.println(far_cycle_level);
            break;
        default:
            break;
        }
        Serial.println("Returning from read...");
        return;
    }
    Serial.println("Finishing up...");
    // always finish with this
    
    // deenergize level probes
    apply_current(eHeliumLevels,0.0);
    
    // update stored levels
    switch (cycle_probe)
    {
    case eHeliumLevelNear:
        levels.near.level = near_cycle_level;
        levels.near.time_since_read = 0;
        levels.near.error = 0;
        last_read_near = millis();
        break;
    case eHeliumLevelFar:
        levels.far.level = far_cycle_level;
        levels.far.time_since_read = 0;
        levels.far.error = 0;
        last_read_far = millis();
        break;
    case eHeliumLevels:
        levels.near.level = near_cycle_level;
        levels.near.time_since_read = 0;
        levels.near.error = 0;
        last_read_near = millis();
        levels.far.level = far_cycle_level;
        levels.far.time_since_read = 0;
        levels.far.error = 0;
        last_read_far = millis();
        break;
    default:
        break;
    }

    cycle_busy = false;
    return;

}

void LHeLevel::set_probe(uint8_t probe)
{
    cycle_probe = probe;
}
bool LHeLevel::toggle_deice()
{
    bdeice = bdeice ? false : true;
    cycle_period = bdeice ? tread + tdeice : tread;
    return bdeice;
}

uint16_t LHeLevel::current_adc(uint8_t id)
{
    Serial.println("Reading IMON...");
    uint16_t out = error;
    switch (id)
    {
    case eHeliumLevelNear:
        Serial.println("Reading near IMON...");
        out = analogRead(nearIMON);
        Serial.print("Near IMON = ");
        break;
    case eHeliumLevelFar:
        Serial.println("Reading far IMON...");
        out = analogRead(farIMON);
        Serial.print("Far IMON = ");
        break;
    default:
        Serial.println("IMON id error...");
        break;
    }
    Serial.println(out);
    return out;
}
uint16_t LHeLevel::level_adc(uint8_t id)
{
    Serial.println("Reading level ADC...");
    uint16_t out = error;
    switch (id)
    {
    case eHeliumLevelNear:
        Serial.println("Reading near ADC...");
        out = analogRead(nearADC);
        break;
    case eHeliumLevelFar:
        Serial.println("Reading far ADC...");
        out = analogRead(farADC);
        break;
    default:
        Serial.println("Read level ADC error...");
        break;
    }
    return out;
}


float LHeLevel::monitor_current(uint8_t id)
{
    uint16_t imon_adc = current_adc(id);
    float out;
    out = 3.3*float(imon_adc)/4095.0;
    out = 1e6*out/Rlim;
    out *= 9./25.;
    return out;

}

float LHeLevel::lhe_level(uint16_t adc_read, uint16_t imon_read)
{
    if(imon_read == 0) return -1.0;
    float imon;
    imon = 3.3*float(imon_read)/4095.0;
    imon = 1e3*imon/Rlim;
    imon *= 9./25.;
    float adc_volts = 3.3*float(adc_read)/4095.0;
    adc_volts *= 3.0;
    float resistance = adc_volts/imon;
    float out = -0.2343*resistance+137.1;
    if(out < 0.0) out = 0.0;
    return out;
}

void LHeLevel::set_level(float level, uint8_t error, uint8_t id)
{
    switch (id)
    {
    case eHeliumLevelNear:
        levels.near.level = level;
        levels.near.error = error;
        levels.near.time_since_read = 0;
        last_read_near = millis(); 
        break;
    case eHeliumLevelFar:
        levels.far.level = level;
        levels.far.error = error;
        levels.far.time_since_read = 0;
        last_read_far = millis(); 
        break;
    default:
        break;
    }
}
float LHeLevel::apply_current(uint8_t id, double current)
{
   uint16_t dacData = current_to_dac(current);
   return apply_current(id,dacData);
}

float LHeLevel::apply_current(uint8_t id, uint16_t dacData)
{
    uint8_t address = addr;
    uint8_t cmd_byte;
    float out = -1.0;
    switch (id)
    {
    case eHeliumLevelNear:
        cmd_byte = 0b00011000;

        dacI2C->beginTransmission(address);
        dacI2C->write(cmd_byte);
        dacI2C->write(dacData >> 8);
        dacI2C->write(dacData);
        dacI2C->endTransmission();

        out = dac_to_current(dacData);
        break;
    case eHeliumLevelFar:
        cmd_byte = 0b00011001;

        dacI2C->beginTransmission(address);
        dacI2C->write(cmd_byte);
        dacI2C->write(dacData >> 8);
        dacI2C->write(dacData);
        dacI2C->endTransmission();
        out = dac_to_current(dacData);
        break;
    case eHeliumLevels:
        cmd_byte = 0b00011111;

        dacI2C->beginTransmission(address);
        dacI2C->write(cmd_byte);
        dacI2C->write(dacData >> 8);
        dacI2C->write(dacData);
        dacI2C->endTransmission();
        out = dac_to_current(dacData);

        break;
    default:
        break;
    }
    return out;
    
}

uint16_t LHeLevel::current_to_dac(float current)
{
    float ilim = 800.0/Rlim;
    if(current < 0) current*=-1;
    if(current > ilim) current = ilim;
    float out_fl = i2adc*current;
    uint16_t out = uint16_t(out_fl);
    out = out << 4;
    return out; 
}

float LHeLevel::dac_to_current(uint16_t dac)
{
    dac = dac >> 4;
    float dac_fl = float(dac);
    float current = dac_fl/i2adc;
    return current;
}