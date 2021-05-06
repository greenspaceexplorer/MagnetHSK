#ifndef ONEWIRETEMP_H_
#define ONEWIRETEMP_H_

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

namespace MagnetHSK{

/** 8 byte address used for OneWire sensors*/
typedef byte (&addr_8)[8];

/**
 * OneWire temperature sensor library for HELIX magnet housekeeping.
 * Used for interfacing with DS18B20 temperature sensors.
 */
class OneWireTemp{
public:
    /** Empty constructor requires temperature sensor address to be specified before reading*/
    OneWireTemp();

    /** Initialize with OneWire bus pin and temperature sensor*/
    OneWireTemp(int,addr_8);
    
    /** Set OneWire bus pin number */
    void set_bus(int);

    /** Get OneWire bus pin number */
    int get_bus(int);

    /** Set temperature sensor address*/
    void set_address(addr_8);

    /** Get temperature sensor address*/
    addr_8 get_address();

    /** Return temperature in bytes */

    /** Return (float) temperature in Celsius */

    /** Return (float) temperature in Fahrenheit */
private:
    int _bus;
    addr_8 _address;

   

}; // OneWireTemp
} // MagnetHSK




#endif // ONEWIRETEMP_H_
