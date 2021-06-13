#include "MagnetRTD.h"


//------------------------------------------------------------------------------

int MagnetRTD::wait_for_process_to_finish(uint8_t chip_select) {
  uint8_t process_finished = 0;
  uint8_t data;
  int goAhead = 1;

  // unsigned long CurrentTime;
  // unsigned long ElapsedTime;

  unsigned long StartTime = millis();

  while (process_finished == 0) {
    data =
        transfer_byte(chip_select, READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
    process_finished = data & 0x40;
    if ((millis() - StartTime) > MaxWaitSpi) {
      goAhead = 0;
      return goAhead;
    }
  }
  // Serial.print(" t=");
  // Serial.print(millis() - StartTime);
  // Serial.print(" ");
  return goAhead;
}

//------------------------------------------------------------------------------

uint8_t MagnetRTD::transfer_byte(uint8_t chip_select, uint8_t ram_read_or_write,
                      uint16_t start_address, uint8_t input_data) {
  uint8_t tx[4], rx[4];

  tx[3] = ram_read_or_write;
  tx[2] = (uint8_t)(start_address >> 8);
  tx[1] = (uint8_t)start_address;
  tx[0] = input_data;
  spi_transfer_block(chip_select, tx, rx, 4);
  return rx[0];
}

//------------------------------------------------------------------------------


uint32_t MagnetRTD::transfer_four_bytes(uint8_t chip_select, uint8_t ram_read_or_write,
                             uint16_t start_address, uint32_t input_data) {
  uint32_t output_data;
  uint8_t tx[7], rx[7];

  tx[6] = ram_read_or_write;
  tx[5] = highByte(start_address);
  tx[4] = lowByte(start_address);
  tx[3] = (uint8_t)(input_data >> 24);
  tx[2] = (uint8_t)(input_data >> 16);
  tx[1] = (uint8_t)(input_data >> 8);
  tx[0] = (uint8_t)input_data;

  spi_transfer_block(chip_select, tx, rx, 7);

  output_data = (uint32_t)rx[3] << 24 | (uint32_t)rx[2] << 16 |
                (uint32_t)rx[1] << 8 | (uint32_t)rx[0];

  return output_data;
}


//------------------------------------------------------------------------------

// Reads and sends a byte array
void MagnetRTD::spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx,
                        uint8_t length) {
  int8_t i;

  output_low(cs_pin); //! 1) Pull CS low

  for (i = (length - 1); i >= 0; i--)
    rx[i] = thisSPI->transfer(tx[i]); //! 2) Read and send byte array

  output_high(cs_pin); //! 3) Pull CS high
}

//------------------------------------------------------------------------------


float MagnetRTD::returnResistance(uint8_t chip_select, uint8_t channel_number) {

  int goAhead;
  goAhead = convert_channel(chip_select, channel_number);
  if (goAhead == 1) {
    int32_t raw_data;
    float voltage_or_resistance_result;
    uint16_t start_address = get_start_address(VOUT_CH_BASE, channel_number);

    raw_data =
        transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);
    voltage_or_resistance_result = (float)raw_data / 1024;
    return voltage_or_resistance_result;
  } else {
    return Nonsense;
  }
}

//------------------------------------------------------------------------------

float MagnetRTD::returnTemperature(uint8_t chip_select, uint8_t channel_number) {
  int goAhead;
  goAhead = convert_channel(chip_select, channel_number);
  if (goAhead == 1) {
    uint32_t raw_data;
    uint8_t fault_data;
    uint16_t start_address =
        get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
    uint32_t raw_conversion_result;
    int32_t signed_data;
    float scaled_result;

    raw_data =
        transfer_four_bytes(chip_select, READ_FROM_RAM, start_address, 0);

    // 24 LSB's are conversion result
    raw_conversion_result = raw_data & 0xFFFFFF;

    signed_data = raw_conversion_result;

    // Convert the 24 LSB's into a signed 32-bit integer
    if (signed_data & 0x800000)
      signed_data = signed_data | 0xFF000000;

    scaled_result = float(signed_data) / 1024;

    return scaled_result;
  } else {
    return -9999;
  }
}

//------------------------------------------------------------------------------

int MagnetRTD::convert_channel(uint8_t chip_select, uint8_t channel_number) {
  // Start conversion
  transfer_byte(chip_select, WRITE_TO_RAM, COMMAND_STATUS_REGISTER,
                CONVERSION_CONTROL_BYTE | channel_number);
  int goAhead;
  goAhead = wait_for_process_to_finish(chip_select);
  return goAhead;
}
