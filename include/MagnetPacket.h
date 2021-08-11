#ifndef MAGNETPACKET_H
#define MAGNETPACKET_H

#include <Arduino.h>
#include <Core_protocol.h>
#include <PacketSerial.h>
#include <driverlib/sysctl.h>

#include <MagnetHSK_protocol.h>
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
#define DOWNBAUD 115200 // Baudrate to the SFC
#define UPBAUD 9600           // Baudrate to upsteam devices
#define TEST_MODE_PERIOD 100  // period in milliseconds between testmode packets being sent

/*******************************************************************************
 * Packet sending/receiving routines
 *******************************************************************************/
void setupPackets(HardwareSerial &downStreamPort);
void periodicPacket();
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

#endif // MAGNETPACKET_H