
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
  uint16_t size = 50;
  int16_t acceleration_Z[50] = {};
  int16_t acceleration_Y[50] = {};
  // create an integer to sum all the values to work out the average
  int32_t sum_Z = 0;
  int32_t sum_Y = 0;
  uint16_t i = 0;
  uint32_t start_time = OSA_TimeGetMsec(); // this is in milliseconds

  for (uint16_t i = 0; i < size; i++) {
    // only care about the Z axis acceleration
    int16_t Z = getSensorDataMMA8451Q_Z_axis();
    int16_t Y = getSensorDataMMA8451Q_Y_axis();

    // want to add all these values to an array
    acceleration_Z[i] = Z;
    acceleration_Y[i] = Y;
    sum_Z = sum_Z + Z;
    sum_Y = sum_Y + Y;
  }
  uint32_t end_time = OSA_TimeGetMsec();
  // now want to look through and see how many oscillations about the average
  // value (equivalent to zero crossings)
  int16_t average_Z = sum_Z / size;
  int16_t average_Y = sum_Y / size;
  warpPrint("%d,%d\n", average_Z, average_Y);
  uint16_t zero_crossing_counter_Z = 0;
  uint16_t zero_crossing_counter_Y = 0;
  uint16_t threshold =
      95; // this is based of 2 standard deviations of the noise
  for (int i = 0; i < size - 1; i++) {
    // number of 0 crossings is double the number of complete sit ups
    if (((acceleration_Y[i] - average_Y) > threshold) &&
            ((acceleration_Y[i + 1] - average_Y) > (threshold * -1)) ||
        (((acceleration_Y[i] - average_Y) > (threshold * -1)) &&
         ((acceleration_Y[i + 1] - average_Y) > threshold))) {
      zero_crossing_counter_Y++;
    }
    if (((acceleration_Z[i] - average_Z) > threshold) &&
            ((acceleration_Z[i + 1] - average_Z) > (threshold * -1)) ||
        (((acceleration_Z[i] - average_Z) > (threshold * -1)) &&
         ((acceleration_Z[i + 1] - average_Z) > threshold))) {
      zero_crossing_counter_Z++;
    }
  }

  warpPrint("%d, %d\n", zero_crossing_counter_Y, zero_crossing_counter_Z);

  // sizeof returns the amount of memory given to the data type. For a array of
  // integers this will be 4 times the number of elements in the array because
  // integers use 4 bytes of memory.

  for (int i = 0; i < size; i++) {
    warpPrint("%d", acceleration_Y[i]);
    warpPrint("\n");
  }
  for (int i = 0; i < size; i++) {
    warpPrint("%d\n", acceleration_Z[i]);
  }
}
