#ifndef MAIN_H_
#define MAIN_H_

// Universal housekeeping includes
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>
#include <Hsk_all_data_types.h>


// Device specific includes
#include <Arduino.h>
// #include <SPI.h>
//#include <LHeLevel.h>

#include <MagnetRTD.h>
// #include <OneWireTemp.h>
//#include <PressureTransducer.h>
//#include <WhisperFlow.h>


//void printAddress(DeviceAddress);
//void printTemperature(DeviceAddress);

// ** Define Magnet HSK board pins **
// *** Flow Meters ***
// RS232 pins
#define DStackFlowRx 3
#define DStackFlowTx 4
#define DShieldFlowRx 5
#define DShieldFlowTx 5
// Analog pins
#define ShieldFlowAIN 17
#define ShieldFlowAOUT 27
#define StackFlowAIN 16
#define StackFlowAOUT 28

// *** LHe Level Probes ***
// DAC Control (I2C)
#define LHeCLR 7
#define LHeSCL 9
#define LHeSDA 10
// Current source monitoring
#define LHe1IMON 23
#define LHe2IMON 24
// Analog read
#define LHe1ADC 15
#define LHe2ADC 16

// *** Magnet RTDs ***
// SPI pins
#define RTDMOSI 8
#define RTDMISO 13
#define RTDCS 12 
#define RTDSCK 11

// *** OneWire temperature probes ***
#define ONEWIREBUS1 38
#define ONEWIREBUS2 37



#endif //MAIN_H_
