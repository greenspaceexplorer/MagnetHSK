#include <Arduino.h>
#include <HelixOneWire.h>

typedef uint8_t TempSensorAddress[8];

void blinkLED(uint8_t LED, uint period);

#define ONE_WIRE_BUS PB_3
HelixOneWire ds(ONE_WIRE_BUS);
TempSensorAddress temp1;
uint8_t devices, tempCount, val;

uint timer = 0, period = 1000;
void setup()
{
    // setup an LED for blinnkery
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(BLUE_LED, LOW);
    pinMode(ONE_WIRE_BUS, INPUT);

    Serial.begin(9600);
}

void loop(void)
{
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius, fahrenheit;

    if (!ds.search(addr))
    {
        Serial.println("No more addresses.");
        Serial.println();
        ds.reset_search();
        delay(250);
        return;
    }

    Serial.print("ROM =");
    for (i = 0; i < 8; i++)
    {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
    }

    if (HelixOneWire::crc8(addr, 7) != addr[7])
    {
        Serial.println("CRC is not valid!");
        return;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0])
    {
    case 0x10:
        Serial.println("  Chip = DS18S20"); // or old DS1820
        type_s = 1;
        break;
    case 0x28:
        Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
    case 0x22:
        Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
    default:
        Serial.println("Device is not a DS18x20 family device.");
        return;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    delay(1000); // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    for (i = 0; i < 9; i++)
    { // we need 9 bytes
        data[i] = ds.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.print(" CRC=");
    Serial.print(HelixOneWire::crc8(data, 8), HEX);
    Serial.println();

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
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
    {
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
    fahrenheit = celsius * 1.8 + 32.0;
    Serial.print("  Temperature = ");
    Serial.print(celsius);
    Serial.print(" Celsius, ");
    Serial.print(fahrenheit);
    Serial.println(" Fahrenheit");
}

// void loop()
// {
//
//     if (millis() % period < timer)
//     {
//         // do something periodically
//
//         val = ds.reset();
//         // val = digitalRead(ONE_WIRE_BUS);
//         if(val){
//             Serial.println("Pulse!");
//         }
//         else{
//             Serial.println("No pulse :(");
//         }
// //         if (!ds.search(temp1))
// //         {
// //             Serial.println("No more addresses...");
// //             Serial.println();
// //             ds.reset_search();
// //             delay(250);
// //             // return;
// //         }
// //         else
// //         {
// //             Serial.println("Found address...");
// //         }
//     }
//     timer = millis() % period;
//     blinkLED(BLUE_LED, 1000);
// }

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
