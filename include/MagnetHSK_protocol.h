/*
 * MagnetHSK_protocol.h
 *
 * Declares the interface protocol for cross device communication.
 *
 * Housekeeping packet consists of header
 * 0-255 payload bytes
 * 1 byte CRCS (or checksum)
 */

/*****************************************************************************
 * Defines
 ****************************************************************************/
#pragma once
#include <stdint.h>

/*******************************************************************************
 * Typedef enums
 *******************************************************************************/

/* Command definitions */
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
  ePressure_regular = 0x1A,
  eHeliumLevels = 0x1B,
  eRTDall = 0x1C,
  eWhisperBoth = 0x1D,
  ePressure = 0x1E,
  eISR = 0xA0,
  eMagField = 0xA1
} MagnetHSK_cmd;
