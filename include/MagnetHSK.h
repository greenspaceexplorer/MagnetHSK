#ifndef MAGNETHSK_H
#define MAGNETHSK_H

#include <Arduino.h>
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
// For magnet housekeeping devices
#include <PressureAndFlow.h>
#include <AnalogPressure.h>
#include <configConstants.h>
#include <configFunctions.h>
#include <supportFunctions.h>
#include <TempSensors.h>

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"
/* other includes you may want go in the src/SUBHSK_lib folder
#include "src/SUBHSK_lib/configuration_constants.h"
*/
// these files above need to be changed based on the thermistor or rtd, etc. So if we borrow the examples from magnet hsk then we can change the channels and types in just these files but keep the function the same.

//////////////////////////////////////
/*
 * CONSTANTS:
 * --PACKETMARKER is defined in Cobs_encoding.h
 * --MAX_PACKET_LENGTH is defined in PacketSerial
 * --NUM_LOCAL_CONTROLS is defined here
 * --FIRST_LOCAL_COMMAND is defined here
 * --TEST_MODE_PERIOD is defined here
 * --BAUD rates are defined here
 *
 */
#define DOWNBAUD 115200       // Baudrate to the SFC
#define UPBAUD 9600           // Baudrate to upsteam devices
#define TEST_MODE_PERIOD 100  // period in milliseconds between testmode packets being sent
#define FIRST_LOCAL_COMMAND 2 // value of hdr->cmd that is the first command local to the board
#define NUM_LOCAL_CONTROLS 32 // how many commands total are local to the board

/*******************************************************************************
 * Magnet housekeeping commands
 *******************************************************************************/
typedef enum MagnetHSK_cmd
{
  // 2-248 are board-specific: these are test commands
  // 2-8 are resistance readings
  eTopStackRTDohms       = 0x02,
  eTopNonStackRTDohms    = 0x03,
  eBottomStackRTDohms    = 0x04,
  eBottomNonStackRTDohms = 0x05,
  eShieldRTD1ohms        = 0x06,
  eShieldRTD2ohms        = 0x07,
  // 9-15 are temperature readings
  eTopStackRTDtemp       = 0x08,
  eTopNonStackRTDtemp    = 0x09,
  eBottomStackRTDtemp    = 0x0A,
  eBottomNonStackRTDtemp = 0x0B,
  eShieldRTD1temp        = 0x0C,
  eShieldRTD2temp        = 0x0D,
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
  eRTDall = 0x1C,
  eWhisperBoth = 0x1D,
  ePressure = 0x1E,
  eMagField = 0x1F,
  eISR = 0xA0,
  eALL = 0xA2
} MagnetHSK_cmd;

/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

/**
 * Initializes magnet housekeeping devices and ports. Returns false if unsuccessful. 
 */
bool initMagnetHSK(); 



/*******************************************************************************
 *  Header only class to read out magnet housekeeping structs
 *******************************************************************************/
 class readMagnetHSK{
     public:
        readMagnetHSK();

 };

/*******************************************************************************
 * Magnet housekeeping structs
 *******************************************************************************/
// subhsk_id=0x01, command associated with this struct: eISR
struct sMagnetInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* Liquid Helium Level Sensors */
// subhsk_id=0x02, command associated with this struct: eHeliumLevels
struct sHeliumLevels {
  uint16_t Farside; //  12 bits ADC, far side level sensor
  uint16_t Nearside; //  12 bits ADC, Near side level sensor
} __attribute__((packed));


/* Magnet RTDs */
// subhsk_id=0x02, command associated with this struct: eRTDall
struct sMagnetRTD {
  float Top_stack; // float, top stack RTD temperature
  float Btm_stack; // float, btm stack RTD temperature
  float Top_nonstack; // float, top non-stack RTD temperature
  float Btm_nonstack; // float, btm non-stack RTD temperature
  float Shield1; // float, shield1 RTD temperature
  float Shield2; // float, shield2 RTD temperature
} __attribute__((packed));

/* Magnet Flowmeters */
// subhsk_id=0x02, command associated with this struct: eFlows
struct sMagnetFlows {
  double stack_pressure; //double, stack flowmeter pressure
  double stack_temperature; //double, stack flowmeter temperature
  double stack_volume; //double, stack flowmeter volume flow
  double stack_mass; //double, stack flowmeter mass flow
  double shield_pressure; //double, shield flowmeter pressure
  double shield_temperature; //double, shield flowmeter temperature
  double shield_volume; //double, shield flowmeter volume flow
  double shield_mass; //double, shield flowmeter mass flow
} __attribute__((packed));

/* Magnet Pressure Transducer */
// subhsk_id=0x02, command associated with this struct: ePressure
struct sMagnetPressure {
  uint16_t Pressure; //0-5 Vdc to ADC., Pressure Transducer pressure reading
} __attribute__((packed));

/* Magnet B-field sensor */
// subhsk_id=0x02, command associated with this struct: eMagField
struct sMagnetField {
    double field; // placeholder for when Noah adds a b-field probe
} __attribute__((packed));


/*******************************************************************************
 * Packet handling functions
 *******************************************************************************/
void checkHdr(const void *sender, const uint8_t *buffer, size_t len);
void forwardDown(const uint8_t *buffer, size_t len, const void *sender);
void checkDownBoundDst(const void *sender);
void badPacketReceived(PacketSerial *sender);
void buildError(housekeeping_err_t *err, housekeeping_hdr_t *respHdr, housekeeping_hdr_t *hdr, int error);
void handlePriority(uint8_t prio_in, uint8_t *responsePacketBuffer);
void setCommandPriority(housekeeping_prio_t *prio, uint8_t *respData, uint8_t len);
int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len, uint8_t *respData);
int handleLocalRead(uint8_t localCommand, uint8_t *buffer);
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t *responsePacketBuffer);
void handleTestMode(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t *responsePacketBuffer);
void switch_LED();

#endif //MAGNETHSK_H