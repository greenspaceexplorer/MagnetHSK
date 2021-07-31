#ifndef MAGNETHSK_H
#define MAGNETHSK_H

#include <Arduino.h>
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
// For magnet housekeeping devices
#include <MagnetWhisper.h>
#include <MagnetRTD.h>
#include <LHeLevel.h>
#include <AnalogPressure.h>
#include <configConstants.h>
#include <configFunctions.h>
#include <supportFunctions.h>
#include <HelixOneWire.h>

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"

/*******************************************************************************
 * Function Declarations
 *******************************************************************************/

void blinkLED(uint8_t LED, uint period);
void serialPrint(HardwareSerial &readPort, HardwareSerial &printPort);
void oneWireTempTest(sTempProbe value, HardwareSerial &printPort);
void printFlow(sMagnetFlow &flow, HardwareSerial &printPort);
void printFlowHdr(String flowMeterName, HardwareSerial &printPort);
void printRtdResist(sMagnetRTDAll &rtds, HardwareSerial &printPort);
void printRtdResistHdr(HardwareSerial &printPort);
void periodicPacket(housekeeping_hdr_t *hsk_header,uint period);

/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

/**
 * Initializes magnet housekeeping devices and ports. Returns false if unsuccessful. 
 */
bool setupMagnetHSK(); 


#endif //MAGNETHSK_H