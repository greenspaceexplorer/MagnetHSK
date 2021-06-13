#ifndef MAGNETWHISPER_H
#define MAGNETWHISPER_H

#include <Arduino.h>
#include <string.h>
#include <MagnetHSK_protocol.h>

class MagnetWhisper{
    public:
        /**
         * Construct MagnetWhisper object. Need to call MagnetWhisper.setup()
         * in the setup() function of main.cpp
         * 
         * Inputs:
         *      port    = Serial port connected to flow meter
         *      timeout = time in milliseconds to wait for a response before
         *                  sending a timeout error
         */
        MagnetWhisper(HardwareSerial &port,uint16_t mytimeout);
        ~MagnetWhisper();

        /**
         * Begins serial communication with flow meter port
         */
        void setup();

        /**
         * Prints current buffer content to given serial port
         */
        void printBuffer(HardwareSerial &printPort);

        /**
         * Sends polling request to flow meter and reads raw data into buffer
         * 
         * Output:
         *      Normally returns the number of bytes read into the buffer
         *      Returns -1 for overflow error 
         *      Returns -2 for timeout error
         *      Returns -3 for incomplete buffer error
         *      Returns -4 for invalid buffer error
         */      
        int readToBuffer();
        
        /**
         * Returns pointer to the internal character buffer
         */
        char* getBuffer();
        
        /**
         * Returns the number of bytes that the buffer can hold
         */
        size_t getBufferSize();
        
        /**
         * Returns the current value of the flowmeter readout
         */
        sMagnetFlow getMagnetFlow();

        /**
         * Sends polling request to flow meter and returns a sWhisperFlow struct
         */
        sMagnetFlow read();
        

    private:
        // serial port
        HardwareSerial *flowPort;
        // character array for polling string
        char poll[3];
        // size of readout buffer
        size_t bufferSize = 100;
        // pointer to readout buffer
        char *buffer;
        // timing variables
        uint16_t timer,timeout;
        unsigned long timeStart;
        // readout status
        int status;
        // flow meter readout struct
        sMagnetFlow magnetFlow;

};

#endif // MAGNETWHISPER_H