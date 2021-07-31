#include <MagnetHSK.h>


/*******************************************************************************
 * printout functions
 *******************************************************************************/
// Prints out flow meter readout in printPort
void printFlow(sMagnetFlow &flow, HardwareSerial &printPort)
{
    String line("");
    String comma(", ");
    line += String(flow.pressure);
    line += comma;
    line += String(flow.temperature);
    line += comma;
    line += String(flow.volume);
    line += comma;
    line += String(flow.mass);
    printPort.print(line);
}
void printFlowHdr(String flowMeterName, HardwareSerial &printPort)
{
    String line("");
    String comma(", ");
    line += flowMeterName;
    line += "_pressure";
    line += comma;
    line += flowMeterName;
    line += "_temp";
    line += comma;
    line += flowMeterName;
    line += "_volflow";
    line += comma;
    line += flowMeterName;
    line += "_massflow";
    printPort.print(line);
}

// Prints out magnet RTD temperatures in printPort
void printRtdResist(sMagnetRTDAll &rtds, HardwareSerial &printPort)
{
    // String units(" Ohms");
    String comma(", ");
    String line("");
    line += String(rtds.top_stack);
    line += comma;
    line += String(rtds.top_nonstack);
    line += comma;
    line += String(rtds.btm_stack);
    line += comma;
    line += String(rtds.btm_nonstack);
    line += comma;
    line += String(rtds.shield1);
    line += comma;
    line += String(rtds.shield2);
    printPort.print(line);
}

void printRtdResistHdr(HardwareSerial &printPort)
{
    printPort.print("ts,tns,bs,bns,s1,s2");
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
 * Testing functions
 *******************************************************************************/
void oneWireTempTest(sTempProbe value, HardwareSerial &printPort){
    if(value.temperature == -9996){
        printPort.println("1wire bus is busy...");
    }
    else if(value.temperature == -9999){
        printPort.println("Probe not found...");
    }
    else if(value.temperature == -9998){
        printPort.println("CRC error...");
    }
    else if(value.temperature == -9997){
        printPort.println("Device type error...");
    }
    else{
        float temp = (float)value.temperature/16.0;
        printPort.print("Temperature = ");
        printPort.print(temp);
        printPort.println(" deg C");
    }

}

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