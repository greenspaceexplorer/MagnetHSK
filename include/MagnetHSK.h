#ifndef MAGNETHSK_H
#define MAGNETHSK_H

#include <Arduino.h>
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
/* These are device specific */
#include <MagnetHSK_protocol.h>
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
 * Magnet housekeeping functions 
 *******************************************************************************/

/**
 * Initializes magnet housekeeping devices and ports. Returns false if unsuccessful. 
 */
bool initMagnetHSK(); 

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