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
#include <TemperatureProbe.h>

#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_flash.h"
#include "driverlib/sysctl.h"


/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

/**
 * Initializes magnet housekeeping devices and ports. Returns false if unsuccessful. 
 */
bool setupMagnetHSK(); 


#endif //MAGNETHSK_H