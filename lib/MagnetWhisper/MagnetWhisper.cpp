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
}

MagnetWhisper::~MagnetWhisper(){
    delete buffer;
}
////////////////////////////////////////////////////////////////////////////////
void MagnetWhisper::setup()
{
    flowPort->begin(9600);
}

////////////////////////////////////////////////////////////////////////////////

void MagnetWhisper::printBuffer(HardwareSerial &printPort){
    printPort.println(buffer);
}

////////////////////////////////////////////////////////////////////////////////

int MagnetWhisper::getRaw()
{
    // Wait for outgoing serial data to finish
    flowPort->flush();
    delay(100);
    // Send request
    flowPort->write(poll);
    delay(100);
    // start timeout timer
    timeStart = millis();
    timer = 0;
    // try reading out until we hit the timeout
    while(timer < timeout){
        // begin reading once a reasonable amount of data is available
        if(flowPort->available()){
            // readout as long as data is available and the buffer isn't full
            uint i = 0;
            while( i < bufferSize && flowPort->available()){
                // read into buffer
                buffer[i] = flowPort->read();
                i++;
            }
            if(i > bufferSize){
                // return overflow error
                return -1;
            }
            else{
                // return number of bytes read into the buffer
                return i-1;
            }
        }
        timer = millis() - timeStart;
    }
    return -2;
    
}
