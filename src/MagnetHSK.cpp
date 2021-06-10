#include <MagnetHSK.h>



/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

bool setupMagnetHSK()
{

  // initialize flow meters
  setupMagnetWhispers();
  // set adc resolution to value for pressure transducer
  // TODO: make sure adc resolution is universal across microcontroller
  return true;
}
