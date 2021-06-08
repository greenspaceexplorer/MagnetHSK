# Magnet Housekeeping 
The magnet housekeeping software is used to control and query magnet related sensors. It is to be installed on the magnet housekeeping microcontroller which, like other housekeeping boards, is a TIVA TM4C123GXL from Texas Instruments. The board sends/receives RS232 packet information to/from the main housekeeping board using the core protocol defined for all housekeeping devices. 

## Installation
Since the TI Tiva libraries hate my macbook, I switched to the PlatformIO tool to manage the magnet housekeeping firmware. It is easier to install, easier to use, and easier to understand (for me, at least!). There are many IDE options available, but the directions here will apply only to the core command line interface, which is included in all IDE options, or can be installed separately. 

- Install the [PlatformIO Core CLI](https://docs.platformio.org/en/latest//core/installation.html) or your flavor of IDE extension.
- Navigate to the home directory of this repository in your terminal
- Connect your Tiva Launchpad
- `pio run`
- `pio run -t upload`

## Magnet Housekeeping Commands
The magnet housekeeping commands are defined in 

## Devices

### Magnet Resistance Temperature Detectors

### Liquid Helium Level Sensors

### Stack Pressure Transducer

### Whisper Flow Meters

### External OneWire Temperature Sensors

### Magnetic Field Sensor

TBD
