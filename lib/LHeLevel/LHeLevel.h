#ifndef LHELEVEL_H
#define LHELEVEL_H

#include <MagnetHSK_protocol.h>
#include <Arduino.h>

class LHeLevel{

    public:
    LHeLevel();
    ~LHeLevel();
    
    uint16_t read();

    private:
       uint16_t level; 
       uint16_t noerror = 65535;


};
#endif