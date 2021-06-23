    //***** // flow meters
    //***** eWhisperStack = 0x0E,
    //***** eWhisperShield = 0x0F,
    //***** eWhisperBoth = 0x1D,
    //***** // external temperature probes
    //***** eTempProbe1 = 0x10,
    //***** eTempProbe2 = 0x11,
    //***** eTempProbe3 = 0x12,
    //***** eTempProbe4 = 0x13,
    //***** eTempProbe5 = 0x14,
    //***** eTempProbe6 = 0x15,
    //***** eTempProbe7 = 0x16,
    //***** eTempProbe8 = 0x17,
    //***** eTempProbe9 = 0x18,
    //***** eTempProbe10 = 0x19,
    //***** eTempProbeAll = 0x20,
    //***** // pressure sensor(s)
    //***** ePressure = 0x1E, // 4-20 mA GP:50 gauge
    //***** ePressureAlt = 0x1A, // heise dxd gauge
    //***** // LHe level probes
    //***** eHeliumLevels = 0x1B,
    //***** // magnetic field sensor
    //***** eMagField = 0x1F,
    //***** // HSK board temperature
    //***** eISR = 0xA0,
    //***** // All available readings
    //***** eALL = 0xA2,
    //***** // Test commands
    //***** eTest = 0x91
/*
 * Main execution script for magnet housekeeping 
 * 
 * Initiates serial port & follows HSK protocol for command responses and error
 * reporting. This program can be used on other devices by changing the device
 * address (myID) and the downStream serial connection (direct line to the SFC)
 */

#include <Arduino.h>
#include <SPI.h>
#include <MagnetHSK_protocol.h>
#include <MagnetHSK.h>
#include <MagnetPacket.h>

/*******************************************************************************
 * Function Declarations
 *******************************************************************************/

void blinkLED(uint8_t LED, uint period);
void serialPrint(HardwareSerial &readPort, HardwareSerial &printPort);
void printFlow(sMagnetFlow &flow, HardwareSerial &printPort);
void printRTDtemps(sMagnetRTDAll &rtds, HardwareSerial &printPort);
void printRTDresists(sMagnetRTDAll &rtds, HardwareSerial &printPort);
void periodicPacket(housekeeping_hdr_t *hsk_header,uint period);

/*******************************************************************************
 * Global Variables 
 *******************************************************************************/
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
uint8_t numDevices = 0; // Keep track of how many devices are upstream
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
};     // Each command's priority takes up one byte

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
uint32_t magnetrtds_a[6] = {0};
sMagnetFlows *magnetflows;
uint64_t magnetflows_a[64] = {0};
sMagnetPressure *magnetpressure;
uint64_t magnetpressure_a[2] = {0};

const int thermalPin = 38;

/* SENSOR STATES AND VARS */
uint32_t TempRead = 0;
// for Launchpad LED example of timer used for reading sensors without holding uC attention
bool is_high = true;

// Declarations for periodic packet
uint8_t packet_fake[5] = {0}; // array for using existing functions to implement command and send packet.
housekeeping_hdr_t *packet_fake_hdr = (housekeeping_hdr_t *)packet_fake; // fakehdr is best way to send a packet

/* MAGNET HOUSEKEEPING GLOBAL VARS */

// level probes
LHeLevel levelsNear_device;
LHeLevel levelsFar_device;
sHeliumLevels  lheLevels;

// gp50 analog pressure transducer
AnalogPressure gp50(160, A0, 12);

// flow meters
uint16_t timeout = 100;
MagnetWhisper stackFlow_device(Serial5, timeout);
MagnetWhisper shieldFlow_device(Serial1, timeout);
sMagnetFlow stackFlow;
sMagnetFlow shieldFlow;
sMagnetFlows bothFlow;

// internal RTDs
SPIClass RTDSPI(0);
uint8_t RTDSPI_SCK = 11;
uint8_t RTDSPI_CS  = 12;
uint8_t RTDSPI_MOSI = 8;
uint8_t RTDSPI_MISO = 13;

MagnetRTD magnetRTD_device(&RTDSPI, RTDSPI_SCK, RTDSPI_CS);
sMagnetRTDAll magnetRTDAll;
sMagnetRTD magnetRTD;

// OneWire temperature probes
TemperatureProbe tempProbe_device;
sTempProbeAll tempProbeAll;
sTempProbe tempProbe;

// Set output serial port
HardwareSerial &serialOut = Serial; // computer (DEBUG)
// HardwareSerial &serialOut = Serial3; // MainHSK

// Setup for reading all active devices
sMagnetAll magnetAll;

/*******************************************************************************
* Main program
*******************************************************************************/
void setup()
{
    // initialize packet communication
    setupPackets(serialOut); 

    // initialize flow meters
    stackFlow_device.setup();
    shieldFlow_device.setup();
    
    // initialize magnet RTDs
    magnetRTD_device.setup();
    

    // setup an LED for blinnkery
    pinMode(GREEN_LED, OUTPUT);
    digitalWrite(GREEN_LED, LOW);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(BLUE_LED, LOW);

    analogReadResolution(gp50.getADCbits());
    
    // serialOut.println("\n***RESTART***"); // DEBUG
}

void loop()
{
    // Blink an LED so we know the board is running
    blinkLED(BLUE_LED,1000);

    packet_fake_hdr->dst = myID;
    packet_fake_hdr->src = eSFC;
    packet_fake_hdr->len = 0;         // this should always be 0, especially because the array is just enough to hold the header.
    // packet_fake_hdr->cmd = eTest;
    packet_fake_hdr->cmd = eRTDallOhms; // which command you want on the timer goes here.

    periodicPacket(packet_fake_hdr,3000);
    
    /* PacketSerial.update() reads and processes incoming packets.
      Returns 0 if it successfully processed the packet.
      Returns nonzero error code if it does not. */
    // if (downStream1.update() != 0)
    // {
    //     // Sends out an error packet if incoming packet was not able to be successfully processed.
    //     badPacketReceived(&downStream1);
    // }
}

/*******************************************************************************
 * Testing functions
 *******************************************************************************/

// blinks an LED with given period
void blinkLED(uint8_t LED, uint period)
{
    static uint BlinkUpdateTime = 0;
    static bool state = true;

    if (millis() % period < BlinkUpdateTime)
    {
        if (state)
        {
            state = !state;
            digitalWrite(LED, LOW);
        }
        else
        {
            state = !state;
            digitalWrite(LED, HIGH);
        }
    }
    BlinkUpdateTime = millis() % period;
}
// Prints out flow meter readout in printPort
void printFlow(sMagnetFlow &flow, HardwareSerial &printPort)
{
    String pressure("\nPressure = ");
    pressure += String(flow.pressure);
    pressure += String(" psia");
    printPort.println(pressure);

    String temp("Temperature = ");
    temp += String(flow.temperature);
    temp += String(" deg C");
    printPort.println(temp);

    String vol("Volumetric Flow = ");
    vol += String(flow.volume);
    vol += String(" slpm");
    printPort.println(vol);

    String mass("Mass Flow = ");
    mass += String(flow.mass);
    mass += String(" slpm");
    printPort.println(mass);
}

// Prints out magnet RTD temperatures in printPort
void printRTDtemps(sMagnetRTDAll &rtds, HardwareSerial &printPort)
{
    String units(" deg C");

    String ts("\nTop Stack = ");
    ts += String(rtds.top_stack);
    ts += units;
    printPort.println(ts);
    
    String tns("Top Non Stack = ");
    tns += String(rtds.top_nonstack);
    tns += units;
    printPort.println(tns);

    String bs("Bottom Stack = ");
    bs += String(rtds.btm_stack);
    bs += units;
    printPort.println(bs);

    String bns("Bottom Non Stack = ");
    bns += String(rtds.btm_nonstack);
    bns += units;
    printPort.println(bns);
    
    String s1("Shield 1 = ");
    s1 += String(rtds.shield1);
    s1 += units;
    printPort.println(s1);
    
    String s2("Shield 2 = ");
    s2 += String(rtds.shield1);
    s2 += units;
    printPort.println(s2);
}

// Prints out magnet RTD temperatures in printPort
void printRTDresists(sMagnetRTDAll &rtds, HardwareSerial &printPort)
{
    String units(" Ohms");

    String ts("\nTop Stack = ");
    ts += String(rtds.top_stack);
    ts += units;
    printPort.println(ts);
    
    String tns("Top Non Stack = ");
    tns += String(rtds.top_nonstack);
    tns += units;
    printPort.println(tns);

    String bs("Bottom Stack = ");
    bs += String(rtds.btm_stack);
    bs += units;
    printPort.println(bs);

    String bns("Bottom Non Stack = ");
    bns += String(rtds.btm_nonstack);
    bns += units;
    printPort.println(bns);
    
    String s1("Shield 1 = ");
    s1 += String(rtds.shield1);
    s1 += units;
    printPort.println(s1);
    
    String s2("Shield 2 = ");
    s2 += String(rtds.shield1);
    s2 += units;
    printPort.println(s2);
}

// Prints out anything from readPort into printPort
void serialPrint(HardwareSerial &readPort, HardwareSerial &printPort)
{
    if (readPort.available())
    {
        uint8_t readout = readPort.read();
        printPort.print(readout);
        printPort.print(" ");
        serialPrint(readPort, printPort);
    }
}
/*******************************************************************************
 * Packet sending/receiving routines
 *******************************************************************************/
void setupPackets(HardwareSerial &downStreamPort)
{
    // Serial port for downstream to Main HSK
    downStreamPort.begin(DOWNBAUD);
    downStream1.setStream(&downStreamPort);
    downStream1.setPacketHandler(&checkHdr);
    // Point to data in a way that it can be read as a header
    hdr_out = (housekeeping_hdr_t *)outgoingPacket;
    hdr_err = (housekeeping_err_t *)(outgoingPacket + hdr_size);
    currentPacketCount = 0;
}

void periodicPacket(housekeeping_hdr_t *hsk_header,uint period)
{
    static uint PacketUpdateTime = 0;

    // send packet unprompted every period
    if (millis() % period < PacketUpdateTime)
    {
        handleLocalCommand(hsk_header, (uint8_t *)hsk_header + hdr_size, (uint8_t *)outgoingPacket);
    }
    PacketUpdateTime = millis() % period;
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
            {
                // THIS IS WHERE A LOCAL COMMAND CONSTRUCTS AND SENDS A NEW PACKET
                handleLocalCommand(hdr_in, (uint8_t *)hdr_in + hdr_size, (uint8_t *)outgoingPacket);
            }
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
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eTopNonStackRTDohms:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eBottomStackRTDohms:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eBottomNonStackRTDohms:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eShieldRTD1ohms:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eShieldRTD2ohms:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    /* Temperature measurements */
    case eTopStackRTDcels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eTopNonStackRTDcels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eBottomStackRTDcels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eBottomNonStackRTDcels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eShieldRTD1cels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eShieldRTD2cels:
    {
        magnetRTD = magnetRTD_device.readResist(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTD, sizeof(magnetRTD));
        retval = (int)sizeof(magnetRTD);
        break;
    }
    case eRTDallOhms:
    {
        magnetRTDAll = magnetRTD_device.readAll(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTDAll, sizeof(magnetRTDAll));
        retval = sizeof(magnetRTDAll);
        break;
    }
    case eRTDallCels:
    {
        magnetRTDAll = magnetRTD_device.readAll(localCommand);
        memcpy(buffer, (uint8_t *)&magnetRTDAll, sizeof(magnetRTDAll));
        retval = sizeof(magnetRTDAll);
        break;
    }

    /* Flow readings */
    case eWhisperStack:
    {
        stackFlow = stackFlow_device.read();
        memcpy(buffer, (uint8_t *)&stackFlow, sizeof(stackFlow));
        retval = (int)sizeof(stackFlow);
        break;
    }
    case eWhisperShield:
    {
        shieldFlow = shieldFlow_device.read();
        memcpy(buffer, (uint8_t *)&shieldFlow, sizeof(shieldFlow));
        retval = (int)sizeof(shieldFlow);
        break;
    }
    /* Temperature probes */
    case eTempProbe1:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(1, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe2:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(2, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe3:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(3, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe4:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(4, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe5:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(5, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe6:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(6, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe7:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(7, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe8:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(8, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe9:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(9, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbe10:
    {
        tempProbe = tempProbe_device.read(localCommand);
        memcpy(buffer, (uint8_t *)&tempProbe, sizeof(tempProbe));
        retval = (int)sizeof(tempProbe);
//        float TempRead = tempSensorVal(10, thermalPin);
//        memcpy(buffer, (uint8_t *)&TempRead, sizeof(TempRead));
//        retval = (int)sizeof(TempRead);
        break;
    }
    case eTempProbeAll:
    {
        tempProbeAll = tempProbe_device.readAll();
        memcpy(buffer, (uint8_t *)&tempProbeAll, sizeof(tempProbeAll));
        retval = (int)sizeof(tempProbeAll);
        break;
    }
    /* Pressure Readings */
    case ePressureAlt:
    {
        break;
    }
    case eHeliumLevels:
        ADCLHeFar[0] = analogRead(A4);
        ADCLHeNear[0] = analogRead(A5);
        helium->far = (uint16_t)ADCLHeFar[0];
        helium->near = (uint16_t)ADCLHeNear[0];
        memcpy(buffer, (uint8_t *)helium, sizeof(sHeliumLevels));
        retval = sizeof(sHeliumLevels);
        break;
    case eWhisperBoth:
    {
        bothFlow.stack = stackFlow_device.read();
        bothFlow.shield = shieldFlow_device.read();
        memcpy(buffer, (uint8_t *)&bothFlow, sizeof(bothFlow));
        retval = sizeof(bothFlow);
        break;
    }
    case ePressure:
    {
        uint16_t localPressure = gp50.readADC();
        magnetpressure->Pressure = localPressure;
        memcpy(buffer, magnetpressure, sizeof(sMagnetPressure));
        retval = sizeof(sMagnetPressure);
        break;
    }
    case eISR:
    {
        sHSKBoardTemp boardTemp;
        uint32_t TempRead = analogRead(TEMPSENSOR);
        boardTemp.temperature = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
        memcpy(buffer, (uint8_t *)&boardTemp, sizeof(boardTemp));
        retval = sizeof(boardTemp);
        break;
    }
    case eMagField:
    {
        // TODO: add a magnetic field sensor
        break;
    }
    case eALL:
    {
        bothFlow.stack = stackFlow_device.read();
        bothFlow.shield = shieldFlow_device.read();
        magnetAll.magnetFlows = bothFlow;

        sHSKBoardTemp boardTemp;
        uint32_t TempRead = analogRead(TEMPSENSOR);
        boardTemp.temperature = (float)(1475 - ((2475 * TempRead) / 4096)) / 10;
        magnetAll.hskBoardTemp = boardTemp;
        
        lheLevels.near = levelsNear_device.read();
        lheLevels.far = levelsFar_device.read();
        magnetAll.heliumLevels = lheLevels;
        
        uint16_t localPressure = gp50.readADC();
        magnetpressure->Pressure = localPressure;
        magnetAll.magnetPressure =  *magnetpressure;

        magnetAll.magnetRTDs =  magnetRTD_device.readAll(eRTDallOhms);

        magnetAll.tempProbeAll = tempProbe_device.readAll();

        memcpy(buffer, (uint8_t *)&magnetAll, sizeof(magnetAll));
        retval = sizeof(magnetAll);

        break;
    }
    case eTest:
    {
        char test[] = "\nI'm a testy little packet!\n";
        memcpy(buffer, (uint8_t *)&test, sizeof(test));
        retval = sizeof(test);
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
    if (hdr->len) // write packet if hdr->len is nonzero
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
    else // hdr->len == 0
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