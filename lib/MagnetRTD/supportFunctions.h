#pragma once

#include <SPI.h> // include the SPI library
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_flash.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
////FOR I2C and ADC comms on TM4C///
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inc/hw_i2c.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_pwm.h>
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/udma.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/systick.h>
#include <driverlib/adc.h>
#include <string.h>


#include "configConstants.h"
//#include <chrono>

// ******************************
// SPI specific Support functions
// ******************************
void Initialize_TM4C123();

void InitADC();

int convert_channel_(uint8_t chip_select, uint8_t channel_number);
int wait_for_process_to_finish(uint8_t chip_select);

void read_voltage_or_resistance_results(uint8_t chip_select,
                                        uint8_t channel_number);

uint32_t transfer_four_bytes(uint8_t chip_select, uint8_t ram_read_or_write,
                             uint16_t start_address, uint32_t input_data);
uint8_t transfer_byte(uint8_t chip_select, uint8_t ram_read_or_write,
                      uint16_t start_address, uint8_t input_data);
void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx,
                        uint8_t length);

void assign_channel(uint8_t chip_select, uint8_t channel_number,
                    uint32_t channel_assignment_data);
void write_custom_table(uint8_t chip_select,
                        struct table_coeffs coefficients[64],
                        uint16_t start_address, uint8_t table_length);
uint16_t get_start_address(uint16_t base_address, uint8_t channel_number);

float returnResistance(uint8_t chip_select, uint8_t channel_number);
float returnTemperature(uint8_t chip_select, uint8_t channel_number);
