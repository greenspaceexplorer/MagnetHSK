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