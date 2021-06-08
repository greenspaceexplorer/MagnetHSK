# HELIX Magnet Housekeeping Firmware
The magnet housekeeping firmware is used to control and query magnet related sensors. It is to be installed on the magnet housekeeping microcontroller which, like other housekeeping boards, is a TIVA TM4C123GXL from Texas Instruments. The board sends/receives RS232 packet information to/from the main housekeeping board using the core protocol defined for all housekeeping devices. 

## Installation
Since the TI Tiva libraries hate my macbook, I switched to the open source [PlatformIO](https://platformio.org) tool to manage the magnet housekeeping firmware. It is easier to install, easier to use, and easier to understand (for me, at least!). There are [many IDE options available](https://platformio.org/install/integration), but the directions here will apply only to the core command line interface, which is included in all IDE options, or can be installed separately. 

- Install the [PlatformIO Core CLI](https://docs.platformio.org/en/latest//core/installation.html) or your flavor of IDE extension.
- Connect your Tiva Launchpad
- Navigate to the home directory of this repository in your terminal and run the following commands:
	- `pio run`
	- `pio run -t upload`
- That's it!

## Magnet Housekeeping Commands
The magnet housekeeping command struct is defined in [include/MagnetHSK_protocol.h](include/MagnetHSK_protocol.h). Available commands are:

| #   |   ID   | Priority | Name                     | Description                                                      |
| --- |:------:|:--------:| ------------------------ | ---------------------------------------------------------------- |
| 1   | `0x02` |    0     | `eTopStackRTDohms`       | Top stack RTD reading in ohms                                    |
| 2   | `0x03` |    0     | `eTopNonStackRTDohms`    | Top non-stack RTD reading in ohms                                |
| 3   | `0x04` |    0     | `eBottomStackRTDohms`    | Bottom stack RTD reading in ohms                                 |
| 4   | `0x05` |    0     | `eBottomNonStackRTDohms` | Bottom non-stack RTD reading in ohms                             |
| 5   | `0x06` |    0     | `eShieldRTD1ohms`        | Shield RTD \#1 reading in ohms                                   |
| 6   | `0x07` |    0     | `eShieldRTD2ohms`        | Shield RTD \#2 reading in ohms                                   |
| 7   | `0x08` |    0     | `eTopStackRTDtemp`       | Top stack RTD reading in Celsius                                 |
| 8   | `0x09` |    0     | `eTopNonStackRTDtemp`    | Top non-stack RTD reading in Celsius                             |
| 9   | `0x0A` |    0     | `eBottomStackRTDtemp`    | Bottom stack RTD reading in Celsius                              |
| 10  | `0x0B` |    0     | `eBottomNonStackRTDtemp` | Bottom non-stack RTD reading in Celsius                          |
| 11  | `0x0C` |    0     | `eShieldRTD1temp`        | Shield RTD \#1 reading in Celsius                                |
| 12  | `0x0D` |    0     | `eShieldRTD2temp`        | Shield RTD \#2 reading in Celsius                                |
| 13  | `0x0E` |    0     | `eWhisperStack`          | All pressure, flow, and temp readings from the stack flow meter  |
| 14  | `0x0F` |    0     | `eWhisperShield`         | All pressure, flow, and temp readings from the shield flow meter |
| 15  | `0x10` |    0     | `eTempProbe1`            | OneWire temperature (location TBD)                               |
| 16  | `0x11` |    0     | `eTempProbe2`            | OneWire temperature (location TBD)                               |
| 17  | `0x12` |    0     | `eTempProbe3`            | OneWire temperature (location TBD)                               |
| 18  | `0x13` |    0     | `eTempProbe4`            | OneWire temperature (location TBD)                               |
| 19  | `0x14` |    0     | `eTempProbe5`            | OneWire temperature (location TBD)                               |
| 20  | `0x15` |    0     | `eTempProbe6`            | OneWire temperature (location TBD)                               |
| 21  | `0x16` |    0     | `eTempProbe7`            | OneWire temperature (location TBD)                               |
| 22  | `0x17` |    0     | `eTempProbe8`            | OneWire temperature (location TBD)                               |
| 23  | `0x18` |    0     | `eTempProbe9`            | OneWire temperature (location TBD)                               |
| 24  | `0x19` |    0     | `eTempProbe10`           | OneWire temperature (location TBD)                               |
| 25  | `0x1A` |    0     | `ePressure_regular`      | Magnet pressure sensor                                           |
| 26  | `0x1B` |    0     | `eHeliumLevels`          | Liquid helium levels                                             |
| 27  | `0x1C` |    0     | `eRTDall`                | All internal magnet RTDs in Celsius                              |
| 28  | `0x1D` |    0     | `eWhisperBoth`           | All readings from both flow meters                               |
| 29  | `0x1E` |    0     | `ePressure`              | Magnet pressure sensor                                           |
| 30  | `0x1F` |    0     | `eMagField`              | Magnetic field sensor                                            |
| 31  | `0xA0` |	0	  |	`eISR`					| On board temperature sensor											|
| 32  | `0xA2` |    0     | `eALL`                   | All magnet housekeeping readings                                 |

TODO: define data structures for each command

## Devices

### Magnet Resistance Temperature Detectors

### Liquid Helium Level Sensors

### Stack Pressure Transducer

### Whisper Flow Meters

### External OneWire Temperature Sensors

### Magnetic Field Sensor

TBD
