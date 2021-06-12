#include "MagnetWhisper.h"

/*******************************************************************************
* Whisper flow meter readout class
*******************************************************************************/
MagnetWhisper::MagnetWhisper(HardwareSerial &port,uint16_t mytimeout)
{
    // Transfer variables into class
    timeout = mytimeout;
    flowPort = &port;
    strcpy(poll,"A\r");

    // create character buffer
    buffer = new char[bufferSize];
    
    // instantiate sMagnetFlow struct
    magnetFlow.mass        = 0.;
    magnetFlow.pressure    = 0.;
    magnetFlow.temperature = 0.;
    magnetFlow.volume      = 0.;
}

MagnetWhisper::~MagnetWhisper(){
    delete buffer;
}
//------------------------------------------------------------------------------
void MagnetWhisper::setup()
{
    flowPort->begin(9600);
}

//------------------------------------------------------------------------------

void MagnetWhisper::printBuffer(HardwareSerial &printPort){
    printPort.println(buffer);
}

//------------------------------------------------------------------------------

int MagnetWhisper::readToBuffer()
{
    // Wait for outgoing serial data to finish
    flowPort->flush();
    delay(100);
    // Send request
    flowPort->write(poll);
    // start timeout timer
    timeStart = millis();
    timer = 0;
    // look for data in the serial buffer until we hit the timeout
    while(!flowPort->available()){
        timer = millis() - timeStart;
        if(timer > timeout){
            // timeout error
            return -2;
        }
    }
    // wait for 100 milliseconds while the flow meter finishes writing to the serial buffer
    delay(100);

    // readout as long as data is available and the buffer isn't full
    uint i = 0;
    while( i < bufferSize && flowPort->available()){
        // read into buffer
        buffer[i] = flowPort->read();
        i++;
    }
    if(i > bufferSize){
        // overflow error
        return -1;
    }
    else if(bufferSize < 35){
        // incomplete buffer error
        return -3;
    }
    else if(buffer[0] != 'A' || buffer[i-1] != '\r'){
        // invalid buffer error
        return -4;
    }
    else{
        // return last index of the readout if no errors occurred
        return i-1;
    }
}

//------------------------------------------------------------------------------

char* MagnetWhisper::getBuffer(){
    return buffer;
}

size_t MagnetWhisper::getBufferSize(){
    return bufferSize;
}

sMagnetFlow MagnetWhisper::getMagnetFlow(){
    return magnetFlow;
}

//------------------------------------------------------------------------------

sMagnetFlow MagnetWhisper::read(HardwareSerial &printPort){
    status = this->readToBuffer();
    
    if(status == -1){ // buffer overflow error
        magnetFlow.mass        = -1.;
        magnetFlow.pressure    = -1.;
        magnetFlow.temperature = -1.;
        magnetFlow.volume      = -1.;
        printPort.println(-1);
        return magnetFlow;
    }
    else if(status == -2){ // timeout error
        magnetFlow.mass        = -2.;
        magnetFlow.pressure    = -2.;
        magnetFlow.temperature = -2.;
        magnetFlow.volume      = -2.;
        printPort.println(-2);
        return magnetFlow;
    }
    else if(status == -3){
        magnetFlow.mass        = -3.;
        magnetFlow.pressure    = -3.;
        magnetFlow.temperature = -3.;
        // store size of incomplete buffer for debuf information
        magnetFlow.volume = float(bufferSize); 
        printPort.println(-3);
        return magnetFlow;
    }
    else if(status == -4){ //invalid buffer error
        magnetFlow.mass        = -4.;
        magnetFlow.pressure    = -4.;
        magnetFlow.temperature = -4.;
        magnetFlow.volume      = -4.;
        printPort.println(-4);
        return magnetFlow;
    }
    else{
        // Normal readout case
        // Example buffer:
        // A +014.07 +027.15 +000.19 +000.18  He-40C
        // 123456789012345678901234567890123456789012
        printPort.println("--Normal readout--");
        char *pch;
        pch = strtok(buffer," ");
        String flowStr;
        uint measureCount = 0;
        while(pch != NULL){

            flowStr = String(pch);
            switch (measureCount)
            {
            case 1:
                Serial.print("Pressure = ");
                magnetFlow.pressure = flowStr.toFloat();
                Serial.println(magnetFlow.pressure);
                break;
            case 2:
                Serial.print("Temperature = ");
                magnetFlow.temperature= flowStr.toFloat();
                Serial.println(magnetFlow.temperature);
                break;
            case 3:
                Serial.print("Volumetric Flow = ");
                magnetFlow.volume = flowStr.toFloat();
                Serial.println(magnetFlow.volume);
                break;
            case 4:
                Serial.print("Mass Flow = ");
                magnetFlow.mass= flowStr.toFloat();
                Serial.println(magnetFlow.mass);
                break;
            
            default:
                break;
            }
            measureCount++;
            pch = strtok(NULL," ");

        }
        
        return magnetFlow;
        

        
    }
}