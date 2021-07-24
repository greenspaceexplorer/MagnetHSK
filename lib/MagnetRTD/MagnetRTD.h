#ifndef MAGNETRTD_H
#define MAGNETRTD_H

#include <Arduino.h>
#include <SPI.h>  // include the SPI library
#include <driverlib/uart.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_nvic.h>
#include <inc/hw_flash.h>
#include <driverlib/sysctl.h>

// include functions to configure the LTC2983 chip
#include "configFunctions.h"
#include "supportFunctions.h"

// for structs and commands
#include <MagnetHSK_protocol.h>

//**********************************************************************************************************
// -- HIGH-LOW and CONSTANTS--
//**********************************************************************************************************
#define output_high(pin) digitalWrite(pin, HIGH)
#define output_low(pin) digitalWrite(pin, LOW)

//**********************************************************************************************************
// -- SPI Code related data --
//**********************************************************************************************************
#define Nonsense -9999
#define MaxWaitSpi 200
//**********************************************************************************************************
// -- ADDRESSES --
//**********************************************************************************************************
#define COMMAND_STATUS_REGISTER (uint16_t)0x0000
#define CH_ADDRESS_BASE (uint16_t)0x0200
#define VOUT_CH_BASE (uint16_t)0x0060
#define READ_CH_BASE (uint16_t)0x0010
#define CONVERSION_RESULT_MEMORY_BASE (uint16_t)0x0010
//**********************************************************************************************************
// -- MISC CONSTANTS --
//**********************************************************************************************************
#define WRITE_TO_RAM (uint8_t)0x02
#define READ_FROM_RAM (uint8_t)0x03
#define CONVERSION_CONTROL_BYTE (uint8_t)0x80

#define VOLTAGE (uint8_t)0x01
#define TEMPERATURE (uint8_t)0x02

//enum rtdChannel{
    //TopStack       = 3,
    //TopNonStack    = 6,
    //BottomStack    = 9,
    //BottomNonStack = 12,
    //Shield1        = 16,
    //Shield2        = 20
//};

enum rtdChannel{
    TopStack       = 12,
    TopNonStack    = 9,
    BottomStack    = 3,
    BottomNonStack = 6,
    Shield1        = 16,
    Shield2        = 20
};

class MagnetRTD{
    public:
        MagnetRTD(SPIClass *mySPI, uint8_t clock, uint8_t chip_select);
        ~MagnetRTD();
        
        void setup();
        
        sMagnetRTD readTemp(uint8_t cmd);
        // 9-15 are temperature readings
        // eTopStackRTDtemp = 0x08
        // eTopNonStackRTDtemp = 0x09
        // eBottomStackRTDtemp = 0x0A
        // eBottomNonStackRTDtemp = 0x0B
        // eShieldRTD1temp = 0x0C
        // eShieldRTD2temp = 0x0D

        sMagnetRTD readResist(uint8_t cmd);
        // 2-8 are resistance readings
        // eTopStackRTDohms = 0x02
        // eTopNonStackRTDohms = 0x03
        // eBottomStackRTDohms = 0x04
        // eBottomNonStackRTDohms = 0x05
        // eShieldRTD1ohms = 0x06
        // eShieldRTD2ohms = 0x07
        
        sMagnetRTDAll readAll(uint8_t cmd);
        // eRTDallOhms = 0x20
        // eRTDallCels = 0x1C



    private:
        // functions
        float returnResistance(uint8_t chip_select, uint8_t channel_number);

        float returnTemperature(uint8_t chip_select, uint8_t channel_number);

        int wait_for_process_to_finish(uint8_t chip_select);

        uint16_t get_start_address(uint16_t base_address, uint8_t channel_number);

        bool is_number_in_array(uint8_t number, uint8_t *array, uint8_t array_length);

        uint8_t transfer_byte(uint8_t chip_select, uint8_t ram_read_or_write,
                              uint16_t start_address, uint8_t input_data);

        uint32_t transfer_four_bytes(uint8_t chip_select, uint8_t ram_read_or_write,
                                     uint16_t start_address, uint32_t input_data);

        void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx,
                                uint8_t length);

        int convert_channel(uint8_t chip_select, uint8_t channel_number);
        
        // variables
        uint8_t clk;
        uint8_t cs;
        SPIClass *thisSPI;
};
#endif // MAGNETRTD_H