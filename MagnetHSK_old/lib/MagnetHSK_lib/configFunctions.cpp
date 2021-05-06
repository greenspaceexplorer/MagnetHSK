#include "configFunctions.h"
// *****************
// Configuration functions copied directly from the evalprom c-generated code
// *****************

void configure_channels() {
  uint8_t channel_number;
  uint32_t channel_assignment_data;

  // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 2, channel_assignment_data);
  // ----- Channel 3: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__2 |
      THERMISTOR_SINGLE_ENDED |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 3, channel_assignment_data);
  // ----- Channel 5: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 5, channel_assignment_data);
  // ----- Channel 6: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__5 |
      THERMISTOR_SINGLE_ENDED |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 6, channel_assignment_data);
  // ----- Channel 8: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 8, channel_assignment_data);
  // ----- Channel 9: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__8 |
      THERMISTOR_SINGLE_ENDED |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 9, channel_assignment_data);
  // ----- Channel 11: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 11, channel_assignment_data);
  // ----- Channel 12: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__11 |
      THERMISTOR_SINGLE_ENDED |
      THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 12, channel_assignment_data);
  // ----- Channel 14: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 14, channel_assignment_data);
  // ----- Channel 16: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__14 |
      THERMISTOR_DIFFERENTIAL | THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 16, channel_assignment_data);
  // ----- Channel 18: Assign Sense Resistor -----
  channel_assignment_data =
      SENSOR_TYPE__SENSE_RESISTOR |
      (uint32_t) 0x106800 
          << SENSE_RESISTOR_VALUE_LSB; // sense resistor - value: 1050.
  assign_channel(CHIP_SELECT, 18, channel_assignment_data);
  // ----- Channel 20: Assign Thermistor Custom Table -----
  channel_assignment_data =
      SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE | THERMISTOR_RSENSE_CHANNEL__18 |
      THERMISTOR_DIFFERENTIAL | THERMISTOR_EXCITATION_MODE__SHARING_ROTATION |
      THERMISTOR_EXCITATION_CURRENT__100UA |
      (uint32_t)0x0
          << THERMISTOR_CUSTOM_ADDRESS_LSB | // thermistor - custom address: 0.
      (uint32_t)0x22 << THERMISTOR_CUSTOM_LENGTH_1_LSB; // thermistor - custom
                                                        // length-1: 34.
  assign_channel(CHIP_SELECT, 20, channel_assignment_data);
}

void configure_memory_table() {
  uint16_t start_address;
  uint16_t table_length;
  // int i;

  // -- Channel 3 custom table --
  table_coeffs ch_3_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_3_coefficients, start_address,
                     table_length);
  // -- Channel 6 custom table --
  table_coeffs ch_6_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_6_coefficients, start_address,
                     table_length);
  // -- Channel 9 custom table --
  table_coeffs ch_9_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_9_coefficients, start_address,
                     table_length);
  // -- Channel 12 custom table --
  table_coeffs ch_12_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_12_coefficients, start_address,
                     table_length);
  // -- Channel 16 custom table --
  table_coeffs ch_16_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_16_coefficients, start_address,
                     table_length);
  // -- Channel 20 custom table --
  table_coeffs ch_20_coefficients[] = {
      {25, 4301},     // -- 1.552, 4.2
      {26, 10240},    // -- 1.65, 10.0
      {29, 15360},    // -- 1.83, 15.0
      {35, 20480},    // -- 2.17, 20.0
      {45, 25600},    // -- 2.83, 25.0
      {61, 30720},    // -- 3.8, 30.0
      {98, 40960},    // -- 6.12, 40.0
      {149, 51200},   // -- 9.3, 50.0
      {208, 61440},   // -- 13.0, 60.0
      {272, 71680},   // -- 17.0, 70.0
      {322, 79053},   // -- 20.1, 77.2
      {341, 81920},   // -- 21.3, 80.0
      {413, 92160},   // -- 25.8, 90.0
      {480, 102400},  // -- 30.0, 100.0
      {547, 112640},  // -- 34.2, 110.0
      {616, 122880},  // -- 38.5, 120.0
      {680, 133120},  // -- 42.5, 130.0
      {746, 143360},  // -- 46.6, 140.0
      {811, 153600},  // -- 50.7, 150.0
      {877, 163840},  // -- 54.8, 160.0
      {944, 174080},  // -- 59.0, 170.0
      {1008, 184320}, // -- 63.0, 180.0
      {1072, 194560}, // -- 67.0, 190.0
      {1139, 204800}, // -- 71.2, 200.0
      {1202, 215040}, // -- 75.1, 210.0
      {1264, 225280}, // -- 79.0, 220.0
      {1328, 235520}, // -- 83.0, 230.0
      {1392, 245760}, // -- 87.0, 240.0
      {1454, 256000}, // -- 90.9, 250.0
      {1518, 266240}, // -- 94.9, 260.0
      {1581, 276480}, // -- 98.8, 270.0
      {1600, 279552}, // -- 100.0, 273.0
      {1643, 286720}, // -- 102.7, 280.0
      {1709, 296960}, // -- 106.8, 290.0
      {1768, 307200}  // -- 110.5, 300.0
  };
  start_address = (uint16_t)592; // Real address = 6*0 + 0x250 = 592
  table_length = (uint8_t)35;    // Real table length = 34 + 1 = 35
  write_custom_table(CHIP_SELECT, ch_20_coefficients, start_address,
                     table_length);
}

void configure_global_parameters() {
  // -- Set global parameters
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xF0,
                TEMP_UNIT__C | REJECTION__50_60_HZ);
  // -- Set any extra delay between conversions (in this case, 0*100us)
  transfer_byte(CHIP_SELECT, WRITE_TO_RAM, 0xFF, 0);
}
