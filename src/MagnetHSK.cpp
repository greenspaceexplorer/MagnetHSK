#include <MagnetHSK.h>



/*******************************************************************************
 * Magnet housekeeping functions 
 *******************************************************************************/

bool initMagnetHSK()
{

  // initialize flow meters
  initMagnetWhispers();
  // set adc resolution to value for pressure transducer
  // TODO: make sure adc resolution is universal across microcontroller
  return true;
}
