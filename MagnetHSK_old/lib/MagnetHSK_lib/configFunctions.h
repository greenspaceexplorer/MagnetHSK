#pragma once

// clang-format off
#include <SPI.h> // include the SPI library
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_flash.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"

#include "configConstants.h"
#include "supportFunctions.h"

// clang-format on
// ******************************
// ALL support functions
// ******************************
void configure_channels();
void configure_memory_table();
void configure_global_parameters();
