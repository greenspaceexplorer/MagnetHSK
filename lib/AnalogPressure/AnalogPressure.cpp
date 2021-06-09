#include "AnalogPressure.h"

////////////////////////////////////////////////////////////////////////////////
AnalogPressure::AnalogPressure(uint16_t rSense, uint8_t adcPin, uint8_t adcBits){
   _adcBits = adcBits;
    _adcPin = adcPin;
    _rSense = rSense;
    
    // scale down to max possible ADC value with given sense resistor
    _adcMax = pow(2.,double(_adcBits))*0.02*float(_rSense);

    // 4 mA/20 mA = 0.2
    _adcMin = 0.2*_adcMax;
}

////////////////////////////////////////////////////////////////////////////////
AnalogPressure::~AnalogPressure(){}

////////////////////////////////////////////////////////////////////////////////
uint16_t AnalogPressure::readADC(){
    return analogRead(_adcPin);
}

////////////////////////////////////////////////////////////////////////////////
float AnalogPressure::readPressure(uint16_t adcVal){
    return _slope*(float(adcVal)-_adcMin);
}

////////////////////////////////////////////////////////////////////////////////
float AnalogPressure::readPressure(){
    uint16_t adcVal = this->readADC();
    float out = readPressure(adcVal);
    if(out < 0.0){
        this->recalibrate(adcVal);
        return 0.0; // TODO: include a warning code that the measurement has been recalibrated
    }
    return out;
}

////////////////////////////////////////////////////////////////////////////////
void AnalogPressure::recalibrate(uint16_t adcVal){
    _adcMin = float(adcVal);
    _slope = _pMax/(_adcMax - _adcMin);
}

