/*
 * Main execution script for magnet housekeeping 
 * 
 * Initiates serial port & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the downStream serial connection (direct line to the SFC)
 */

#include <Arduino.h>
#include <SPI.h>
#include <MagnetHSK.h>

/* Declare instances of PacketSerial to set up the serial lines */
PacketSerial downStream1; // mainHSK

/* Name of this device */
housekeeping_id myID = eMagnetHsk; // CHANGE THIS TO THE ID OF SUBHSK Board

/* Outgoing buffer, for up or downstream. Only gets used once a complete packet
 * is received -- a command or forward is executed before anything else happens,
 * so there shouldn't be any over-writing here. */
uint8_t outgoingPacket[MAX_PACKET_LENGTH] = {0};

/* Use pointers for all device's housekeeping headers and the autopriorityperiods*/
housekeeping_hdr_t *hdr_in;
housekeeping_hdr_t *hdr_out;
housekeeping_err_t *hdr_err;
housekeeping_prio_t *hdr_prio;
/* Memory buffers for housekeeping system functions */
uint8_t numDevices = 0;                                                                                                                // Keep track of how many devices are upstream
uint8_t commandPriority[NUM_LOCAL_CONTROLS] = {
                                              0, // eTopStackRTDohms 
                                              0, // eTopNonStackRTDohms
                                              0, // eBottomStackRTDohms
                                              0, // eBottomNonStackRTDohms
                                              0, // eShieldRTD1ohms
                                              0, // eShieldRTD2ohms
                                              0, // eTopStackRTDtemp
                                              0, // eTopNonStackRTDtemp
                                              0, // eBottomStackRTDtemp
                                              0, // eBottomNonStackRTDtemp
                                              0, // eShieldRTD1temp
                                              0, // eShieldRTD2temp
                                              0, // eWhisperStack
                                              0, // eWhisperShield
                                              0, // eTempProbe1
                                              0, // eTempProbe2 
                                              0, // eTempProbe3 
                                              0, // eTempProbe4 
                                              0, // eTempProbe5 
                                              0, // eTempProbe6 
                                              0, // eTempProbe7 
                                              0, // eTempProbe8 
                                              0, // eTempProbe9 
                                              0, // eTempProbe10
                                              0, // ePressureRegular
                                              0, // eHeliumLevels
                                              0, // eRTDall
                                              0, // eWhisperBoth
                                              0, // ePressure
                                              0, // eISR
                                              0, // eMagField
                                              0  // eALL
                                              }; // Each command's priority takes up one byte

PacketSerial *serialDevices = &downStream1;
uint8_t addressList = 0; // List of all downstream devices

/* Utility variables for internal use */
size_t hdr_size = sizeof(housekeeping_hdr_t) / sizeof(hdr_out->src); // size of the header
uint8_t numSends = 0;                                                // Used to keep track of number of priority commands executed
uint16_t currentPacketCount = 0;
// static_assert(sizeof(float) == 4);
unsigned long timelastpacket; //for TestMode

/* Variables for readout of magnet sensors */
uint32_t ADCLHeFar[1];
uint32_t ADCLHeNear[1];
sHeliumLevels *helium;
uint16_t helium_a[2] = {0};
sMagnetRTD *magnetrtds;
uint32_t magnetrtds_a[6] = {0};
sMagnetFlows *magnetflows;
uint64_t magnetflows_a[64] = {0};
sMagnetPressure *magnetpressure;
uint64_t magnetpressure_a[2] = {0};

const int thermalPin = 38;

/* SENSOR STATES AND VARS */
#define SENSOR_UPDATE_PERIOD 1000 // how often to check/write sensors
uint32_t TempRead = 0;
// for Launchpad LED example of timer used for reading sensors without holding uC attention
#define LED GREEN_LED
#define LED_UPDATE_PERIOD 1350
uint LEDUpdateTime = 0; // keeping LED to visualize no hanging
bool is_high = true;
#define PACKET_UPDATE_PERIOD 10050
uint PacketUpdateTime = 0; // unprompted packet timer
uint8_t packet_fake[5] = {0};       // array for using existing functions to implement command and send packet.

/* Magnet housekeeping global vars */

AnalogPressure gp50(160,A0,10); // gp50 analog pressure transducer

// Debugging stuff
int byteme = 0;

/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
  // to DEBUG this device when connecting directly to computer, use this serial port instead of Serial.1
  //Serial.begin(DOWNBAUD);
  //downStream1.setStream(&Serial);
  //downStream1.setPacketHandler(&checkHdr);

  // Serial port for downstream to Main HSK
  Serial3.begin(DOWNBAUD);
  downStream1.setStream(&Serial3);
  downStream1.setPacketHandler(&checkHdr);
  // Point to data in a way that it can be read as a header
  hdr_out = (housekeeping_hdr_t *)outgoingPacket;
  hdr_err = (housekeeping_err_t *)(outgoingPacket + hdr_size);
  currentPacketCount = 0;
  PacketUpdateTime = millis() + PACKET_UPDATE_PERIOD;

  // LED on launchpad
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  
  // initialize magnet housekeeping
  initMagnetHSK();
  
  analogReadResolution(gp50.getADCbits());
//  Serial3.print("Analog read resolution = ");
//  Serial3.println(gp50.getADCbits());
  Serial3.println();
  Serial3.println("*****RESTART*****");
}

/*******************************************************************************
 * Main program
 ******************************************************************************/
void loop()
{

  if (millis()%LED_UPDATE_PERIOD < LEDUpdateTime)
  {
    switch_LED(); 
    TempRead = analogRead(TEMPSENSOR);
  }
  LEDUpdateTime = millis()%LED_UPDATE_PERIOD;

  // example send packet unprompted every PACKET_PERIOD
  if (millis()%PACKET_UPDATE_PERIOD < PacketUpdateTime)
  {
    housekeeping_hdr_t *packet_fake_hdr = (housekeeping_hdr_t *)packet_fake; // fakehdr is best way to send a packet
    packet_fake_hdr->dst = myID;
    packet_fake_hdr->src = eSFC;
    packet_fake_hdr->len = 0;   // this should always be 0, especially because the array is just enough to hold the header.
    packet_fake_hdr->cmd = ePressure; // which command you want on the timer goes here.
    // to construct a packet, pass it a fake header
    handleLocalCommand(packet_fake_hdr, (uint8_t *)packet_fake_hdr + hdr_size, (uint8_t *)outgoingPacket);
    
  }
  PacketUpdateTime = millis()%PACKET_UPDATE_PERIOD;

  if(Serial3.available()){
    Serial3.println("Receiving bytes...");
    while(Serial3.available() > 0){
      byteme = Serial3.read();
      Serial3.print(byteme);
    }

  }
  /* Continuously read in one byte at a time until a packet is received */
  //if (downStream1.update() != 0)
  //  badPacketReceived(&downStream1);
}


/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

bool initMagnetHSK(){
  
  // initialize flow meters
  initMagnetWhispers();
  // set adc resolution to value for pressure transducer
  // TODO: make sure adc resolution is universal across microcontroller
  return true;
}


/*******************************************************************************
 * Packet handling functions
 *******************************************************************************/
void checkHdr(const void *sender, const uint8_t *buffer, size_t len)
{
  // Default header & error data values
  hdr_out->src = myID; // Source of data packet
  hdr_in = (housekeeping_hdr_t *)buffer;
  hdr_prio = (housekeeping_prio_t *)(buffer + hdr_size);
  // If an error occurs at this device from a message
  if (hdr_in->dst == eBroadcast || hdr_in->dst == myID)
    hdr_err->dst = myID;
  else
    hdr_err->dst = hdr_in->dst;
  // If the checksum didn't match, throw a bad args error
  // Check for data corruption
  if (!(verifyChecksum((uint8_t *)buffer)))
  {
    //error_badArgs(hdr_in, hdr_out, hdr_err);
    buildError(hdr_err, hdr_out, hdr_in, EBADARGS);
    fillChecksum((uint8_t *)outgoingPacket);
    downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
    currentPacketCount++;
  }
  else
  {
    // Check if the message is a broadcast or local command and only then execute it.
    if (hdr_in->dst == eBroadcast || hdr_in->dst == myID)
    {
      if (hdr_in->cmd == eTestMode)
        handleTestMode(hdr_in, (uint8_t *)hdr_in + hdr_size, (uint8_t *)outgoingPacket);
      else if ((int)(hdr_in->cmd < 254) && (int)(hdr_in->cmd > 249))
        handlePriority(hdr_in->cmd, (uint8_t *)outgoingPacket); // for doing a send of priority type.
      else
        handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size, (uint8_t *)outgoingPacket); // this constructs the outgoingpacket when its a localcommand and sends the packet.
    }
    // If the message wasn't meant for this device pass it along (up is away from SFC and down and is to SFC
    else
      forwardDown(buffer, len, sender);
  }
}
// forward downstream to the SFC
void forwardDown(const uint8_t *buffer, size_t len, const void *sender)
{
  downStream1.send(buffer, len);
  checkDownBoundDst(sender);
  currentPacketCount++;
}

/* checkDownBoundDst Function flow:
 * --Checks to see if the downstream device that sent the message is known
 *    --If not, add it to the list of known devices
 *    --If yes, just carry on
 * --Executed every time a packet is received from downStream
 * 
 * Function params:
 * sender:    PacketSerial instance (serial line) where the message was received
 * 
 */
void checkDownBoundDst(const void *sender)
{
  if (serialDevices == (PacketSerial *)sender)
  {
    if (addressList == 0)
    {
      addressList = (uint8_t)hdr_in->src;
      numDevices++;
      return;
    }
  }
}
/* Function flow:
 * --Find the device address that produced the error
 * --Execute the bad length function & send the error to the SFC
 * Function params:
 * sender:    PacketSerial instance which triggered the error protocol
 * Send an error if a packet is unreadable in some way */
void badPacketReceived(PacketSerial *sender)
{
  if (sender == serialDevices)
  {
    hdr_in->src = addressList;
  }
  hdr_out->src = myID;
  buildError(hdr_err, hdr_out, hdr_in, EBADLEN);
  fillChecksum((uint8_t *)outgoingPacket);
  downStream1.send(outgoingPacket, hdr_size + hdr_out->len + 1);
  currentPacketCount++;
}

// Function for building the error packets to send back when an error is found (see the Core_Protocol.h for the defs of the errors and the error typdefs).
void buildError(housekeeping_err_t *err, housekeeping_hdr_t *respHdr, housekeeping_hdr_t *hdr, int error)
{
  respHdr->cmd = eError;
  respHdr->len = 4;
  err->src = hdr->src;
  err->dst = hdr->dst;
  err->cmd = hdr->cmd;
  err->error = error;
}

/*******************************************************************************
 * END OF Packet handling functions
 *******************************************************************************/

// sending priority command function
// probably can be cleaned up
// Note: SendAll is 253 and SendLow is 250 so we made SendLow-> int priority=1 for checking the device's list of command's priorities.
// got a priority request from destination dst
void handlePriority(uint8_t prio_in, uint8_t *responsePacketBuffer)
{
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + hdr_size;
  int priority = 0;
  int retval = 0;
  uint8_t sum = 0; // hdr length of data atatched from all those commands data
                   //  respHdr->cmd = hdr_in->cmd;
  // priority == 4 when this function is called is code for "eSendAll"
  // otherwise priority=1,2,3 and that maps to eSendLowPriority+priority
  if (prio_in == eSendAll)
    priority = 4;
  else
    priority = prio_in - 249;
  //  int retval;
  respHdr->src = myID;
  respHdr->dst = eSFC;
  respHdr->cmd = prio_in;
  // go through every priority
  for (int i = 0; i < NUM_LOCAL_CONTROLS; i++)
  {
    if (commandPriority[i] == (uint8_t)priority || priority == 4)
    {
      retval = handleLocalRead((uint8_t)i + FIRST_LOCAL_COMMAND, respData + sum);
      // if that read overflowed the data???? fix later?
      sum += (uint8_t)retval;
    }
    else
      sum += 0;
  }
  respHdr->len = sum;
  fillChecksum(responsePacketBuffer);
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1);
  currentPacketCount++;
}

// function for when a "SetPriority" command is received by this device, adding that commands priority value to the array/list
void setCommandPriority(housekeeping_prio_t *prio, uint8_t *respData, uint8_t len)
{
  //  housekeeping_prio_t * set_prio = (housekeeping_prio_t *) prio;
  commandPriority[prio->command - FIRST_LOCAL_COMMAND] = (uint8_t)prio->prio_type;
  memcpy(respData, (uint8_t *)prio, len);
}
// Fn to handle a local command write.
// This gets called when a local command is received
// with data (len != 0).
int handleLocalWrite(uint8_t localCommand, uint8_t *data, uint8_t len, uint8_t *respData)
{
  int retval = 0;
  switch (localCommand)
  {
  case eSetPriority:
  {
    setCommandPriority((housekeeping_prio_t *)data, respData, len);
    retval = len;
    break;
  }
  default:
    retval = EBADCOMMAND;
    break;
  }
  return retval;
}

// Fn to handle a local command read.
// This gets called when a local command is received
// with no data (len == 0)
// buffer contains the pointer to where the data
// will be written.
// int returns the number of bytes that were copied into
// the buffer, or EBADCOMMAND if there's no command there
int handleLocalRead(uint8_t localCommand, uint8_t *buffer)
{
  int retval = 0;
  switch (localCommand)
  {
  case ePingPong:
    retval = 0;
    break;
  case eSetPriority:
    retval = EBADLEN;
    break;
  /* Resistance measurements */
  case eTopStackRTDohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  case eTopNonStackRTDohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  case eBottomStackRTDohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  case eBottomNonStackRTDohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  case eShieldRTD1ohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  case eShieldRTD2ohms:
  {
    float ResRead = returnResistance(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&ResRead, sizeof(ResRead));
    retval = (int)sizeof(ResRead);
    break;
  }
  /* Temperature measurements */
  case eTopStackRTDtemp:
  {
    // topstack
    float TempRead = returnTemperature(CHIP_SELECT, 3);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = sizeof(TempRead);
    break;
  }
  case eTopNonStackRTDtemp:
  {
    // topnonstack
    float TempRead = returnTemperature(CHIP_SELECT, 6);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = sizeof(TempRead);
    break;
  }
  case eBottomStackRTDtemp:
  {
    // bottom stack
    float TempRead = returnTemperature(CHIP_SELECT, 9);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eBottomNonStackRTDtemp:
  {
    // bottom nonstack
    float TempRead = returnTemperature(CHIP_SELECT, 12);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eShieldRTD1temp:
  {
    float TempRead = returnTemperature(CHIP_SELECT, 16);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eShieldRTD2temp:
  {
    float TempRead = returnTemperature(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  /* Flow readings */
  case eWhisperStack:
  {
    double gasdata[4];
    char gastype[100];
    char errorcode[100];
    bool FlowRead = readFlow(1, gasdata, gastype, errorcode);
    uint8_t flowread_ft;
    if (FlowRead)
    {
      flowread_ft = 0;
      memcpy(buffer, (uint8_t *)&gasdata, sizeof(gasdata));
      memcpy(buffer + sizeof(gasdata), (uint8_t *)&gastype, sizeof(gastype));
      memcpy(buffer + sizeof(gasdata) + sizeof(gastype), &flowread_ft, sizeof(flowread_ft));
      retval = (sizeof(gasdata) + sizeof(gastype) + sizeof(flowread_ft));
    }
    else
    {
      flowread_ft = 1;
      memcpy(buffer, (uint8_t *)&errorcode, sizeof(errorcode));
      memcpy(buffer + sizeof(errorcode), &flowread_ft, sizeof(flowread_ft));
      retval = (sizeof(errorcode) + sizeof(flowread_ft));
    }
    break;
  }
  case eWhisperShield:
  {
    double gasdata[4];
    char gastype[100];
    char errorcode[100];
    bool FlowRead = readFlow(2, gasdata, gastype, errorcode);
    uint8_t flowread_ft;
    if (FlowRead)
    {
      flowread_ft = 0;
      memcpy(buffer, (uint8_t *)&gasdata, sizeof(gasdata));
      memcpy(buffer + sizeof(gasdata), (uint8_t *)&gastype, sizeof(gastype));
      memcpy(buffer + sizeof(gasdata) + sizeof(gastype), &flowread_ft, sizeof(flowread_ft));
      retval = (sizeof(gasdata) + sizeof(gastype) + sizeof(flowread_ft));
    }
    else
    {
      flowread_ft = 1;
      memcpy(buffer, (uint8_t *)&errorcode, sizeof(errorcode));
      memcpy(buffer + sizeof(errorcode), &flowread_ft, sizeof(flowread_ft));
      retval = (sizeof(errorcode) + sizeof(flowread_ft));
    }
    break;
  }
  /* Temperature probes */
  case eTempProbe1:
  {
    float TempRead = tempSensorVal(1, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe2:
  {
    float TempRead = tempSensorVal(2, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe3:
  {
    float TempRead = tempSensorVal(3, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe4:
  {
    float TempRead = tempSensorVal(4, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe5:
  {
    float TempRead = tempSensorVal(5, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe6:
  {
    float TempRead = tempSensorVal(6, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe7:
  {
    float TempRead = tempSensorVal(7, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe8:
  {
    float TempRead = tempSensorVal(8, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe9:
  {
    float TempRead = tempSensorVal(9, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  case eTempProbe10:
  {
    float TempRead = tempSensorVal(10, thermalPin);
    memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
    retval = (int)sizeof(TempRead);
    break;
  }
  /* Pressure Readings */
  case ePressureRegular:
  {
    double pressureValue;
    double temperature;
    char typeOfPressure;
    char errorcode[100];
    bool pressure_bool = readPressureSensor(&pressureValue, &temperature, &typeOfPressure, errorcode);
    uint8_t pressure_ft;
    if (pressure_bool)
    {
      pressure_ft = 0;
      memcpy(buffer, (uint8_t *)&pressureValue, sizeof(pressureValue));
      memcpy(buffer + sizeof(pressureValue), (uint8_t *)&temperature, sizeof(temperature));
      memcpy(buffer + sizeof(pressureValue) + sizeof(temperature), (uint8_t *)&typeOfPressure, sizeof(typeOfPressure));
      retval = (sizeof(pressureValue) + sizeof(temperature) + sizeof(typeOfPressure) + sizeof(pressure_ft));
    }
    else
    {
      pressure_ft = 1;
      memcpy(buffer, (uint8_t *)&errorcode, sizeof(errorcode));
      memcpy(buffer + sizeof(errorcode), &pressure_ft, sizeof(pressure_ft));
      retval = (sizeof(errorcode) + sizeof(pressure_ft));
    }
    break;
  }
  case eHeliumLevels:
    ADCLHeFar[0] = analogRead(A4);
    ADCLHeNear[0] = analogRead(A5);
    helium->Farside = (uint16_t)ADCLHeFar[0];
    helium->Nearside = (uint16_t)ADCLHeNear[0];
    memcpy(buffer, (uint8_t *)helium, sizeof(sHeliumLevels));
    retval = sizeof(sHeliumLevels);
    break;
  case eRTDall:
    //hdr_out->src=eMagnetHsk;
    magnetrtds->Top_stack = returnTemperature(CHIP_SELECT, 3);
    magnetrtds->Btm_stack = returnTemperature(CHIP_SELECT, 9);
    magnetrtds->Top_nonstack = returnTemperature(CHIP_SELECT, 6);
    magnetrtds->Btm_nonstack = returnTemperature(CHIP_SELECT, 12);
    magnetrtds->Shield1 = returnTemperature(CHIP_SELECT, 16);
    magnetrtds->Shield2 = returnTemperature(CHIP_SELECT, 20);
    memcpy(buffer, (uint8_t *)magnetrtds, sizeof(sMagnetRTD));
    retval = sizeof(sMagnetRTD);
    break;
  case eWhisperBoth:
  {
    double gasdata[4];
    char gastype[32];
    char errorcode[32];
    bool FlowRead = readFlow(1, gasdata, gastype, errorcode);
    if (FlowRead)
    {
      magnetflows->stack_pressure = gasdata[0];
      magnetflows->stack_temperature = gasdata[1];
      magnetflows->stack_volume = gasdata[2];
      magnetflows->stack_mass = gasdata[3];
    }
    else
    {
      magnetflows->stack_pressure = -9999;
      magnetflows->stack_temperature = -9999;
      magnetflows->stack_volume = -9999;
      magnetflows->stack_mass = -9999;
    }
    FlowRead = readFlow(2, gasdata, gastype, errorcode);
    if (FlowRead)
    {
      magnetflows->shield_pressure = gasdata[0];
      magnetflows->shield_temperature = gasdata[1];
      magnetflows->shield_volume = gasdata[2];
      magnetflows->shield_mass = gasdata[3];
    }
    else
    {
      magnetflows->shield_pressure = -9999;
      magnetflows->shield_temperature = -9999;
      magnetflows->shield_volume = -9999;
      magnetflows->shield_mass = -9999;
    }
    memcpy(buffer, (uint8_t *)magnetflows, sizeof(sMagnetFlows));
    retval = sizeof(sMagnetFlows);
    break;
  }
  case ePressure:
  {

    // Serial3.println(gp50.readADC());
    magnetpressure->Pressure = 190;
    // analogReadResolution(10);
    // Serial3.print("Magnet pressure 10 bit ADC = ");
    // Serial3.println(analogRead(A0));
    // analogReadResolution(12);
    // Serial3.print("Magnet pressure 12 bit ADC = ");
    // Serial3.println(magnetpressure->Pressure,DEC);
    memcpy(buffer, (uint8_t *)magnetpressure, sizeof(sMagnetPressure));
    retval = sizeof(sMagnetPressure);
    break;
  }
  case eISR:
  {
    uint32_t TempRead = analogRead(TEMPSENSOR);
    float TempC = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
    memcpy(buffer, (uint8_t *)&TempC, sizeof(TempC));
    retval = sizeof(TempC);
    break;
  }
  case eMagField:
  {
    // TODO: add a magnetic field sensor
    break;
  }
  case eALL:
  {
    // should iterate through all commands or make a new struct?
    break;
  }
  case eReset:
  {
    SysCtlReset();
    return 0;
  }
  default:
    retval = EBADCOMMAND;
  }
  return retval;
}

// Function to call first when localcommand sent.
// Store the "result" as retval (which is the bytes read or written, hopefully)
void handleLocalCommand(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t *responsePacketBuffer)
{
  int retval = 0;
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = eSFC;
  if (hdr->len)
  {
    retval = handleLocalWrite(hdr->cmd, data, hdr->len, respData); // retval is negative construct the baderror hdr and send that instead.
    if (retval >= 0)
    {
      //      *respData= 5;
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; // response bytes of the write.
    }
    else
    {
      housekeeping_err_t *err = (housekeeping_err_t *)respData;
      buildError(err, respHdr, hdr, retval);
    }
  }
  else
  {
    // local read. by definition these always go downstream.
    retval = handleLocalRead(hdr->cmd, respData);
    if (retval >= 0)
    {
      respHdr->cmd = hdr->cmd;
      respHdr->len = retval; //bytes read
    }
    else
    {
      housekeeping_err_t *err = (housekeeping_err_t *)respData;
      buildError(err, respHdr, hdr, retval); // the err pointer is pointing to the data of the response packet based on the line above so this fn fills that packet.
    }
  }
  fillChecksum(responsePacketBuffer);
  // send to SFC
  downStream1.send(responsePacketBuffer, respHdr->len + hdr_size + 1);
  currentPacketCount++;
}

void handleTestMode(housekeeping_hdr_t *hdr, uint8_t *data, uint8_t *responsePacketBuffer)
{
  housekeeping_hdr_t *respHdr = (housekeeping_hdr_t *)responsePacketBuffer;
  uint8_t *respData = responsePacketBuffer + sizeof(housekeeping_hdr_t);
  respHdr->src = myID;
  respHdr->dst = hdr->src;
  // if length was actually placed then go into testmode, else build badlength error.
  if (hdr->len)
  {
    //construct data incoming to be the num testpackets and send the data packet in a while loop and decrement numtestpackets?
    uint16_t numTestPackets = ((uint16_t)(*(data + 1) << 8)) | *(data); // figure out the correct way to get 2 bytes into a 16_t
    timelastpacket = millis();
    while (numTestPackets)
    {
      if (long(millis() - timelastpacket) > 0)
      { // only send every 50 milliseconds?
        *(respData) = numTestPackets;
        *(respData + 1) = numTestPackets >> 8;
        respHdr->cmd = hdr->cmd;
        respHdr->len = 0x02; // response bytes of the write.
        fillChecksum(responsePacketBuffer);
        // send to SFC
        downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1);
        numTestPackets--;
        timelastpacket = timelastpacket + TEST_MODE_PERIOD;
        currentPacketCount++;
      }
    }
  }
  else
  {
    housekeeping_err_t *err = (housekeeping_err_t *)respData;
    buildError(err, respHdr, hdr, EBADLEN);
    fillChecksum(responsePacketBuffer);
    // send to SFC
    downStream1.send(responsePacketBuffer, respHdr->len + sizeof(housekeeping_hdr_t) + 1);
    currentPacketCount++;
  }
}
void switch_LED()
{
  if (is_high)
  {
    is_high = false;
    digitalWrite(LED, LOW);
  }
  else
  {
    is_high = true;
    digitalWrite(LED, HIGH);
  }
}
// 