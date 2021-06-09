#ifndef ANALOGPRESSURE_H
#define ANALOGPRESSURE_H

#include <Arduino.h>

/**
 * Provides functionality to read out 4-20 mA analog pressure transducer with 
 * a given sense resistor
 */
class AnalogPressure{
    public:
        /**
         *  Constructor sets adc resolution and calculates slope and intercept 
         * for ADC to PSI conversion
         */
        AnalogPressure(uint16_t rSense, uint8_t adcPin, uint8_t adcBits);
        ~AnalogPressure();

        /**
         *  Returns current ADC value
         */
        uint16_t readADC();
        
        /**
         * Converts given ADC value into PSI
         */
        float readPressure(uint16_t adcVal);

        /**
         * Reads out ADC and converts to PSI
         */
        float readPressure();
        
        /**
         * Returns number of bits to be used for the analogRead ADC
         */
        uint8_t getADCbits(){
            return _adcBits;
        };
        
    private:
        /**
         * Recalibrates the pressure conversion to a new minimum ADC value
         */
        void recalibrate(uint16_t adcVal);

        uint8_t _adcBits;
        uint8_t _adcPin;
        uint16_t _rSense;
        float _adcMax;
        float _adcMin;
        float _slope;
        float _pMax = 25;

};

#endif // ANALOGPRESSURE_H