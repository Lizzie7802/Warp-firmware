
#include <time.h>

#include "config.h"
#include <stdint.h>

#include "fsl_clock_manager.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_mcglite_hal.h"
#include "fsl_misc_utilities.h"
#include "fsl_port_hal.h"
#include "fsl_power_manager.h"
#include "fsl_rtc_driver.h"
#include "fsl_spi_master_driver.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"

#include "devMMA8451Q.h"
#include "devSSD1331.h"

void measure_breathing_rate() {

  configureSensorMMA8451Q(
      0x00, /* Payload: Disable FIFO */
      0x01  /* Normal read 8bit, 800Hz, normal, active mode */
  );        // this allows the values to be changed

  // want to get rid of the offset.

  // create an empty array to store the values in
  uint16_t size = 300;
  uint16_t acceleration[300] = {};
  // create an integer to sum all the values to work out the average
  uint32_t sum = 0;
  uint16_t i = 0;
  uint32_t start_time = OSA_TimeGetMsec(); // this is in milliseconds

  for (uint16_t i = 0; i < size; i++) {
    // only care about the Z axis acceleration
    int16_t Z = getSensorDataMMA8451Q_Z_axis();
    // want to add all these values to an array
    acceleration[i] = Z;
    sum = sum + Z;
    OSA_TimeDelay(50);
  }
  uint32_t end_time = OSA_TimeGetMsec();
  // the sum is 1000 times the average.
  // to smooth the data want to compute a rolling average
  /*
    uint16_t average_acceleration[40] = {};
    for (uint16_t i = 0; i < size; i++) {
      if (i < 2) {
        average_acceleration[i] = acceleration[i] * size;
      } else if (i > size - 2) {
        average_acceleration[i] = acceleration[i] * size;
      } else {
        // want to compute a rolling average
        uint16_t rolling_average = 0;
        for (uint16_t j = 0; j < 4;
             j++) { // this will compute a rolling average over 4 samples
          rolling_average = rolling_average + acceleration[i - 2 + j];
        }
        average_acceleration[i] = rolling_average * size;
      }
    }*/

  // now want to look through and see how many oscillations about the average
  // value (equivalent to zero crossings)
  uint16_t average = sum / size;
  warpPrint("%d\n", average);
  uint16_t counter = 0;
  uint16_t threshold = 100;
  for (uint16_t i = 0; i < size - 1; i++) {
    if (((acceleration[i] - average) > threshold) &&
        ((acceleration[i + 1] - average) < (threshold * -1))) {
      counter = counter + 1;
    }
  }

  warpPrint("%d\n", counter);

  // sizeof returns the amount of memory given to the data type. For a array of
  // integers this will be 4 times the number of elements in the array because
  // integers use 4 bytes of memory.
  for (int i = 0; i < size; i++) {
    warpPrint("%d", acceleration[i]);
    warpPrint("\n");
  }
}
