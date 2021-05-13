/*
 * Hsk_all_data_types.h
 *
 * Declares all the structs to parse the data of hsk packets. 
 * Update when changing the "firmware" commands.
 * To be used by both energia code to store the variables when reading
 * and by the sfc to parse the packets per command. 
 * Example Commands and subhsk id that use the given struct appear in comments above that struct
 *
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>

/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/*****Main Hsk Structs*****/
/* Main launchpad */
// subhsk_id=0x01, command associated with this struct: eISR
struct sMainInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* Flowmeters */
// subhsk_id=0x01, command associated with this struct: eFlows
struct sMainFlows {
  double pressure; //double, flowmeter 1 pressure
  double temperature; //double, flowmeter 1 temperature
  double volume; //double, flowmeter 1 volume flow
  double mass; //double, flowmeter 1 mass flow
  double setpoint;
  double gasnum;
} __attribute__((packed));

/* 1Wire Temperatures */
// subhsk_id=0x01, command associated with this struct: eOneWireBridge_read
struct sMainTemps {
  float temps[10];
} __attribute__((packed));

struct sMainFlowsGasString {
  char response[100];
} __attribute__((packed));

/* Main Pressure Transducer */
// subhsk_id=0x01, command associated with this struct: ePressure
struct sMainPressure {
  float Pressure; //Serial Pressure Transducer pressure reading
  float Temperature; // Serial temperature internal to pressure transducer
} __attribute__((packed));

/* Relay Signals In to ADC */
// subhsk_id=0x01, command associated with this struct: ""
struct sMainRelaysIN {
  uint16_t relay[10]; // these are 12 bit adc signals (or lower depending on the ADC chosen)
} __attribute__((packed));


/*****Magnet Hsk Structs*****/
/* Magnet launchpad */
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
  double pressure; //double, flowmeter 1 pressure
  double temperature; //double, flowmeter 1 temperature
  double volume; //double, flowmeter 1 volume flow
  double mass; //double, flowmeter 1 mass flow
} __attribute__((packed));

/* Magnet Pressure Transducer */
// subhsk_id=0x02, command associated with this struct: ePressure
struct sMagnetPressure {
  uint16_t Pressure; //0-5 Vdc to ADC., Pressure Transducer pressure reading
} __attribute__((packed));

/*****DCT Hsk Structs*****/
/* DCT launchpad */
// subhsk_id=0x01, command associated with this struct: eISR
struct sDCTInternalTemp {
  float Temperature; // internal temperature sensor to uC
} __attribute__((packed));

/* DCT Thermistors */
// subhsk_id=0x03, command associated with this struct: eThermistors
struct sDCTThermistors {
  float Therms[25]; // float, Thermistors temperature
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMon
struct sDCTHV {
  uint16_t CatVmon; // 12 bits ADC, Cathode HV supply Vmon
  uint16_t CatImon; // 12 bits ADC, Cathode HV supply Imon
  uint16_t PotVmon; // 12 bits ADC, Potential HV supply Vmon
  uint16_t PotImon; // 12 bits ADC, Potential HV supply Imon
} __attribute__((packed));

/* DCT Pressure Transducer */
// subhsk_id=0x03, command associated with this struct: ePressure
struct sDCTPressure {
  uint16_t Pressure; // 10 bits ADC, Pressure Transducer pressure reading
} __attribute__((packed));

/* HV supplies monitoring*/
// subhsk_id=0x03, command associated with this struct: eHVMonConverted
struct sDCTHVConverted {
  float CatV; // 12 bits ADC converted to kV-Cathode HV supply
  float CatI; // 12 bits ADC, converted to mA-Cathode HV supply
  float PotV; // 12 bits ADC, converted to kV-Potential HV supply
  float PotI; // 12 bits ADC, converted to mA-Potential HV supply
} __attribute__((packed));


