#ifndef MAGNETWHISPER_H
#define MAGNETWHISPER_H

#include <Arduino.h>
#include <string.h>

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
         */      
        int getRaw();

    private:
        HardwareSerial *flowPort;
        char poll[3];
        size_t bufferSize = 100;
        char *buffer;
        uint16_t timer,timeout;
        unsigned long timeStart;

};

#endif // MAGNETWISHPER_H