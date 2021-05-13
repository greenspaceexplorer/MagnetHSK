#include "TempSensors.h"

// Adapted from OneWire DS18S20, DS18B20, DS1822 Temperature Example
// http://www.pjrc.com/teensy/td_libs_OneWire.html

byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8], reqaddr[8];
float celsius, fahrenheit;
bool valid;
int j;
char chunk;
char command[27];
const byte temp2addr[10][8] = {
    {0x28, 0xF6, 0xBA, 0x79, 0x97, 0x06, 0x03, 0x0D},  //#1
    {0x28, 0x6A, 0xC6, 0x79, 0x97, 0x06, 0x03, 0x8D},  //#2
    {0x28, 0xBB, 0xC3, 0x79, 0x97, 0x06, 0x03, 0xAC},  //#3
    {0x28, 0x99, 0x87, 0x79, 0x97, 0x06, 0x03, 0x82},  //#4(replaced)
    {0x28, 0x81, 0xD3, 0x79, 0x97, 0x06, 0x03, 0xF2},  //#5
    {0x28, 0xC2, 0xBE, 0x79, 0x97, 0x06, 0x03, 0x23},  //#6
    {0x28, 0xBB, 0xC2, 0x79, 0x97, 0x06, 0x03, 0x61},  //#7
    {0x28, 0x0E, 0xA4, 0x79, 0x97, 0x06, 0x03, 0x00},  //#8
    {0x28, 0x7B, 0xAA, 0x79, 0x97, 0x06, 0x03, 0xD1},  //#9
    {0x28, 0xB8, 0x94, 0x79, 0x97, 0x06, 0x03, 0x31}}; //#10

//  OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
float tempSensorVal(int tempNum, int pinNum)
{
  OneWire ds(pinNum);
  ds.reset_search();
  delay(10);
  if (tempNum > 10 || tempNum < 1)
  {
    return -9998;
  }
  j = 0;
  while (j < 100)
  {
    valid = true;
    if (ds.search(addr))
    {
      for (int i = 0; i < 8; i++)
      {
        if (addr[i] != temp2addr[tempNum - 1][i])
        {
          valid = false;
          break;
        }
      }
      if (!valid)
      {
        continue;
      }
      break;
    }
    else
    { // Unable to find this address
      return -9997;
    }
    j++;
  }
  if (j > 99)
  {
    return -9999;
  }
  // the first ROM byte indicates which chip
  switch (addr[0])
  {
  case 0x10:
    type_s = 1;
    break;
  case 0x28:
    type_s = 0;
    break;
  case 0x22:
    type_s = 0;
    break;
  default:
    return -9996; // Unrecognized
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end
  delay(100);       // maybe 750ms is enough, maybe not
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad
  for (i = 0; i < 9; i++)
  { // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s)
  {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10)
    {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  }
  else
  { // This is us
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00)
      raw = raw & ~7; // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20)
      raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40)
      raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  return celsius;
}
