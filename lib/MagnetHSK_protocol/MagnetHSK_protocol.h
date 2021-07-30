#ifndef MAGNETHSK_PROTOCOL
#define MAGNETHSK_PROTOCOL

#include <Arduino.h>
#include <Core_protocol.h>
#include <map>
/*******************************************************************************
 * Magnet housekeeping commands
 *******************************************************************************/

// update these if you change anything below
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board
#define NUM_LOCAL_CONTROLS 37 // how many commands total are local to the board
#define NUM_TEMP_PROBES 10 // number of external OneWire temperature probes

typedef enum MagnetHSK_cmd
{
    // 2-248 are board-specific: these are test commands
    // RTD resistance
    eTopStackRTDohms       = 0x02,
    eTopNonStackRTDohms    = 0x03,
    eBottomStackRTDohms    = 0x04,
    eBottomNonStackRTDohms = 0x05,
    eShieldRTD1ohms        = 0x06,
    eShieldRTD2ohms        = 0x07,
    // RTD temperature
    eTopStackRTDcels       = 0x08,
    eTopNonStackRTDcels    = 0x09,
    eBottomStackRTDcels    = 0x0A,
    eBottomNonStackRTDcels = 0x0B,
    eShieldRTD1cels        = 0x0C,
    eShieldRTD2cels        = 0x0D,
    // get all RTD readings
    eRTDallOhms            = 0x30,
    eRTDallCels            = 0x31,
    // flow meters
    eWhisperStack          = 0x0E,
    eWhisperShield         = 0x0F,
    eWhisperBoth           = 0x1D,
    // external temperature probes
    eTempProbe1            = 0x10,
    eTempProbe2            = 0x11,
    eTempProbe3            = 0x12,
    eTempProbe4            = 0x13,
    eTempProbe5            = 0x14,
    eTempProbe6            = 0x15,
    eTempProbe7            = 0x16,
    eTempProbe8            = 0x17,
    eTempProbe9            = 0x18,
    eTempProbe10           = 0x19,
    eTempProbeAll          = 0x20,
    // pressure sensor(s)
    ePressure              = 0x1E,    // 0-5V GP:50 gauge
    ePressureAlt           = 0x1A, // heise dxd gauge
    // LHe level probes
    eHeliumLevelNear       = 0x21,
    eHeliumLevelFar        = 0x22,
    eHeliumLevels          = 0x1B,
    // magnetic field sensor
    eMagField              = 0x1F,
    // HSK board temperature
    eISR                   = 0xA0,
    // All available readings
    eALL                   = 0xA2,
    // Test commands
    eTest                  = 0x91
} MagnetHSK_cmd;

/*******************************************************************************
 * Magnet housekeeping structs
 *******************************************************************************/
// subhsk_id=0x01, command associated with this struct:
//*****    eISR = 0xA0
struct sHSKBoardTemp
{
    float temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* Liquid Helium Level Sensors */
// subhsk_id=0x02, commands associated with this struct:
//*****    eHeliumLevelNear = 0x21, eHeliumLevelFar = 0x22
struct sHeliumLevel
{
    float level;  //  12 bits ADC, far side level sensor
    uint32_t time_since_read;
    uint8_t error;
} __attribute__((packed));
// subhsk_id=0x02, commands associated with this struct:
//*****    eHeliumLevels = 0x1B,
struct sHeliumLevels
{
    sHeliumLevel near; //  12 bits ADC, Near side level sensor
    sHeliumLevel far;  //  12 bits ADC, far side level sensor
} __attribute__((packed));

/* Magnet RTDs */
// subhsk_id=0x02, commands associated with this struct:
//*****    eTopStackRTDohms = 0x02,
//*****    eTopNonStackRTDohms = 0x03,
//*****    eBottomStackRTDohms = 0x04,
//*****    eBottomNonStackRTDohms = 0x05,
//*****    eShieldRTD1ohms = 0x06,
//*****    eShieldRTD2ohms = 0x07,
//*****    eTopStackRTDcels = 0x08,
//*****    eTopNonStackRTDcels = 0x09,
//*****    eBottomStackRTDcels = 0x0A,
//*****    eBottomNonStackRTDcels = 0x0B,
//*****    eShieldRTD1cels = 0x0C,
//*****    eShieldRTD2cels = 0x0D
struct sMagnetRTD
{
    float value;
} __attribute__((packed));

// subhsk_id=0x02, commands associated with this struct:
//*****    eRTDallOhms = 0x30,
//*****    eRTDallCels = 0x31,
struct sMagnetRTDAll
{
    float top_stack;    // top stack RTD temperature
    float btm_stack;    // btm stack RTD temperature
    float top_nonstack; // top non-stack RTD temperature
    float btm_nonstack; // btm non-stack RTD temperature
    float shield1;      // shield1 RTD temperature
    float shield2;      // shield2 RTD temperature
} __attribute__((packed));

/* Magnet Flowmeters */
// subhsk_id=0x02, commands associated with this struct:
//*****    eWhisperStack = 0x0E,
//*****    eWhisperShield = 0x0F,
struct sMagnetFlow
{
    float pressure;    // flowmeter pressure
    float temperature; // flowmeter temperature
    float volume;      // flowmeter volume flow
    float mass;        // flowmeter mass flow
} __attribute__((packed));

// subhsk_id=0x02, commands associated with this struct:
//*****    eWhisperBoth = 0x1D
struct sMagnetFlows
{
    sMagnetFlow stack;
    sMagnetFlow shield;
} __attribute__((packed));

/* Magnet Pressure Transducers */
// subhsk_id=0x02, commands associated with this struct:
//*****    ePressure = 0x1E, // 4-20 mA GP:50 gauge
struct sMagnetPressure
{
    uint16_t pressure; //0-5 Vdc to ADC., Pressure Transducer pressure reading
} __attribute__((packed));

// subhsk_id=0x02, commands associated with this struct:
//*****    ePressureAlt = 0x1A, // heise dxd gauge
struct sMagnetPressureAlt
{
    float pressure;
    float temperature;
} __attribute__((packed));

/* External OneWire RTDs */
// subhsk_id=0x02, commands associated with this struct:
//*****    eTempProbe1 = 0x10,
//*****    eTempProbe2 = 0x11,
//*****    eTempProbe3 = 0x12,
//*****    eTempProbe4 = 0x13,
//*****    eTempProbe5 = 0x14,
//*****    eTempProbe6 = 0x15,
//*****    eTempProbe7 = 0x16,
//*****    eTempProbe8 = 0x17,
//*****    eTempProbe9 = 0x18,
//*****    eTempProbe10 = 0x19,
struct sTempProbe
{
    uint64_t sn; // serial number
    int16_t temperature;
} __attribute__((packed));

// subhsk_id=0x02, commands associated with this struct:
//*****    eTempProbeAll = 0x20,
struct sTempProbeAll
{
    sTempProbe probe1;
    sTempProbe probe2;
    sTempProbe probe3;
    sTempProbe probe4;
    sTempProbe probe5;
    sTempProbe probe6;
    sTempProbe probe7;
    sTempProbe probe8;
    sTempProbe probe9;
    sTempProbe probe10;
} __attribute__((packed));

/* Magnet B-field sensor */
// subhsk_id=0x02, commands associated with this struct:
//*****    eMagField = 0x1F,
struct sMagnetField
{
    float field; // placeholder for when Noah adds a b-field probe
} __attribute__((packed));

/* Measurements from all installed magnet housekeeping devices */
//*****    eALL = 0xA2,
struct sMagnetAll
{
    sHSKBoardTemp hskBoardTemp;
    sHeliumLevels heliumLevels;
    sMagnetRTDAll magnetRTDs;
    sMagnetFlows magnetFlows;
    sMagnetPressure magnetPressure;
    sTempProbeAll tempProbeAll;
} __attribute__((packed));

//*****    // Test commands
// subhsk_id=0x02, commands associated with this struct:
//*****    eTest = 0x91
struct sTestPacket
{
    char test[28];
} __attribute__((packed));

#endif // MAGNETHSK_PROTOCOL