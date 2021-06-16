#ifndef MAGNETHSK_PROTOCOL
#define MAGNETHSK_PROTOCOL

#include <Arduino.h>
#include <Core_protocol.h>
/*******************************************************************************
 * Magnet housekeeping commands
 *******************************************************************************/
typedef enum MagnetHSK_cmd
{
    // 2-248 are board-specific: these are test commands
    // 2-8 are resistance readings
    eTopStackRTDohms = 0x02,
    eTopNonStackRTDohms = 0x03,
    eBottomStackRTDohms = 0x04,
    eBottomNonStackRTDohms = 0x05,
    eShieldRTD1ohms = 0x06,
    eShieldRTD2ohms = 0x07,
    // 9-15 are temperature readings
    eTopStackRTDcels = 0x08,
    eTopNonStackRTDcels = 0x09,
    eBottomStackRTDcels = 0x0A,
    eBottomNonStackRTDcels = 0x0B,
    eShieldRTD1cels = 0x0C,
    eShieldRTD2cels = 0x0D,
    // 16-18 are flow meter readings
    eWhisperStack = 0x0E,
    eWhisperShield = 0x0F,
    // 16-25 are read temperature probes
    eTempProbe1 = 0x10,
    eTempProbe2 = 0x11,
    eTempProbe3 = 0x12,
    eTempProbe4 = 0x13,
    eTempProbe5 = 0x14,
    eTempProbe6 = 0x15,
    eTempProbe7 = 0x16,
    eTempProbe8 = 0x17,
    eTempProbe9 = 0x18,
    eTempProbe10 = 0x19,
    // 26 is read pressure
    ePressureRegular = 0x1A, // why are there two pressures?
    eHeliumLevels = 0x1B,
    eRTDallOhms = 0x20,
    eRTDallCels = 0x1C,
    eWhisperBoth = 0x1D,
    ePressure = 0x1E,
    eMagField = 0x1F,
    eISR = 0xA0,
    eALL = 0xA2,
    eTest = 0x91
} MagnetHSK_cmd;

/*******************************************************************************
 * Magnet housekeeping structs
 *******************************************************************************/
// subhsk_id=0x01, command associated with this struct: eISR
struct sMagnetInternalTemp
{
    float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* Liquid Helium Level Sensors */
// subhsk_id=0x02, command associated with this struct: eHeliumLevels
struct sHeliumLevels
{
    uint16_t Farside;  //  12 bits ADC, far side level sensor
    uint16_t Nearside; //  12 bits ADC, Near side level sensor
} __attribute__((packed));

/* Magnet RTDs */
// subhsk_id=0x02, command associated with this struct: eRTDall
struct sMagnetRTD
{
    float Top_stack;    // top stack RTD temperature
    float Btm_stack;    // btm stack RTD temperature
    float Top_nonstack; // top non-stack RTD temperature
    float Btm_nonstack; // btm non-stack RTD temperature
    float Shield1;      // shield1 RTD temperature
    float Shield2;      // shield2 RTD temperature
} __attribute__((packed));

/* Magnet Flowmeters */
// subhsk_id=0x02, commands associated with this struct: eWhisperStack, eWhisperShield
struct sMagnetFlow
{
    float pressure;     // flowmeter pressure
    float temperature;  // flowmeter temperature
    float volume;       // flowmeter volume flow
    float mass;         // flowmeter mass flow
} __attribute__((packed));

// subhsk_id=0x02, command associated with this struct: eWhisperBoth
struct sMagnetFlows
{
    sMagnetFlow stack;
    sMagnetFlow shield;
} __attribute__((packed));

/* Magnet Pressure Transducer */
// subhsk_id=0x02, command associated with this struct: ePressure
struct sMagnetPressure
{
    uint16_t Pressure; //0-5 Vdc to ADC., Pressure Transducer pressure reading
} __attribute__((packed));

/* Magnet B-field sensor */
// subhsk_id=0x02, command associated with this struct: eMagField
struct sMagnetField
{
    double field; // placeholder for when Noah adds a b-field probe
} __attribute__((packed));

struct sTestPacket
{
    char test[28];
} __attribute__((packed));

#endif // MAGNETHSK_PROTOCOL