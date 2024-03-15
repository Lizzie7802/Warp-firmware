

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

void sit_up_counter() {

  configureSensorMMA8451Q(
      0x00, /* Payload: Disable FIFO */
      0x01  /* Normal read 8bit, 800Hz, normal, active mode */
  );        // this allows the values to be changed

  // want to get rid of the offset.

  // create an empty array to store the values in
  uint16_t size = 20;
  // create an integer to sum all the values to work out the average
  int32_t sum_Z = 0;
  int32_t sum_Y = 0;
  uint32_t start_time = OSA_TimeGetMsec(); // this is in milliseconds
  int16_t peaks[5] = {};
  int16_t troughs[5] = {};
  int16_t highest_Y = 0;
  int16_t lowest_Y = 0;
  int16_t crossing_counter_Y = 0;
  int16_t max_noise_Y = 0;
  int16_t min_noise_Y = 0;
  int32_t average_noise_Y = 0;
  int32_t noise_counter = 0;
  uint16_t sit_up_counter_Y = 0;
  uint8_t state = 0;
  warpPrint("Beginning calibration \n Stay still");
  while (OSA_TimeGetMsec() - start_time < 1000) {
    // this is calibration were the user should stay still to estimate the noise
    // which can then be used for calibration
    int16_t Y = getSensorDataMMA8451Q_Y_axis();
    if (Y > max_noise_Y) {
      max_noise_Y = Y;
    } else if (Y < min_noise_Y) {
      min_noise_Y = Y;
    }
    average_noise_Y = average_noise_Y + Y;
    noise_counter++;
    OSA_TimeDelay(10);
    warpPrint(".");
  }
  warpPrint("Ready!");
  warpPrint("\n %d \n", noise_counter);
  average_noise_Y = average_noise_Y / noise_counter;
  warpPrint("%d,\n", average_noise_Y);
  max_noise_Y = (max_noise_Y - average_noise_Y) * 2;
  min_noise_Y = (min_noise_Y - average_noise_Y) * 2;
  warpPrint("%d,%d \n", max_noise_Y, min_noise_Y);
  int16_t previous_value_Y = getSensorDataMMA8451Q_Y_axis() - average_noise_Y;
  int32_t end_calibration = OSA_TimeGetMsec();
  // do rolling average
  while (OSA_TimeGetMsec() - end_calibration < 20000) { // record for 20 seconds
    sum_Z = 0;
    sum_Y = 0;

    for (uint16_t i = 0; i < size; i++) {
      // only care about the Z axis acceleration
      int16_t Y = getSensorDataMMA8451Q_Y_axis() - average_noise_Y;

      // want to add all these values to an array to be average over
      sum_Y = sum_Y + Y;
    }
    int16_t average_Y = sum_Y / size;
    warpPrint("%d\n",
              average_Y); // this is the rolling average at this value
    if ((average_Y > max_noise_Y) || (average_Y < min_noise_Y)) {
      // this means the reading is above the noise level so something is
      // happening can use the highest and lowest values to determine
      // confidence.
      if ((average_Y > highest_Y) && (state == 1)) {
        highest_Y = average_Y;
      } else if ((average_Y < lowest_Y) && (state == 0)) {
        lowest_Y = average_Y;
      }
      // now want to work out if there is a crossing over the gravity threshold
      int16_t up_threshold = -1800;

      int16_t down_threshold = -600;
      if ((state == 0) && (average_Y < up_threshold)) {
        // reached peak from lying down
        // change to state 1
        state = 1;
      }
      if ((state == 1) && (average_Y > down_threshold)) {
        // going back down
        state = 0;
        sit_up_counter_Y++;
        warpPrint("Sit up!");
        highest_Y = 0;
        lowest_Y = 0;
      }
    }

    previous_value_Y = average_Y;
  }
  warpPrint("%d", sit_up_counter_Y);
}
