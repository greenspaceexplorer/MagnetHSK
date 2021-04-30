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
typedef enum MagnetHSK_cmd {
  // 2-248 are board-specific: these are test commands
  // 2-8 are resistance readings
  eResistanceCh3 = 0x02, // bottom stack side
  eResistanceCh6 = 0x03, // bottom non-stack side
  eResistanceCh9 = 0x04, // top stack side
  eResistanceCh12 = 0x05, // top non-stack side
  eResistanceCh16 = 0x06, // shield sensor #1
  eResistanceCh20 = 0x07, // shield sensor #2
  // 9-15 are temperature readings
  eTempCh3 = 0x08,
  eTempCh6 = 0x09,
  eTempCh9 = 0x0A,
  eTempCh12 = 0x0B,
  eTempCh16 = 0x0C,
  eTempCh20 = 0x0D,
  // 16-18 are flow readings
  eWhisperVflow1 = 0x0E,
  eWhisperMflow1 = edit,
  eWhisperTemp1  = edit,
  eWhisperPress1 = edit,

  eWhisperVflow2 = 0x0E,
  eWhisperMflow2 = edit,
  eWhisperTemp2  = edit,
  eWhisperPress2 = edit,

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
  eFlows = 0x1D, 
  ePressure = 0x1E,
  eISR = 0xA0,
} MagnetHSK_cmd;
