

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

int16_t probability(int16_t difference_down, int16_t difference_up,
                    int16_t stddev) {
  int16_t probability_of_correct = 1;
  if (difference_down > 3 * stddev) {
    if (difference_up > 3 * stddev) {
      probability_of_correct = 100;
    } else if (difference_up > 2 * stddev) {
      probability_of_correct = 95;
    } else if (difference_up > stddev) {
      probability_of_correct = 68;
    } else {
      probability_of_correct = 32;
    }
  } else if (difference_down > 2 * stddev) {
    if (difference_up > 3 * stddev) {
      probability_of_correct = 95;
    } else if (difference_up > 2 * stddev) {
      probability_of_correct = 90; // 0.95*0.95
    } else if (difference_up > stddev) {
      probability_of_correct = 65; // 0.95*0.68
    } else {
      probability_of_correct = 30; // 0.95*0.32
    }
  } else if (difference_down > stddev) {
    if (difference_up > 3 * stddev) {
      probability_of_correct = 68; // 0.68* 1
    } else if (difference_up > 2 * stddev) {
      probability_of_correct = 64; // 0.68*0.95
    } else if (difference_up > stddev) {
      probability_of_correct = 46; // 0.68*0.68
    } else {
      probability_of_correct = 22; // 0.68*0.32
    }
  } else {
    if (difference_up > 3 * stddev) {
      probability_of_correct = 32; // 0.32* 1
    } else if (difference_up > 2 * stddev) {
      probability_of_correct = 30; // 0.32*0.95
    } else if (difference_up > stddev) {
      probability_of_correct = 22; // 0.32*0.68
    } else {
      probability_of_correct = 10; // 0.32*0.32
    }
  }
  return probability_of_correct;
}

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
  int16_t highest_Z = 0;
  int16_t lowest_Z = 0;
  int16_t crossing_counter_Y = 0;
  int16_t crossing_counter_Z = 0;
  int16_t max_noise_Y = 0;
  int16_t min_noise_Y = 0;
  int16_t max_noise_Z = 0;
  int16_t min_noise_Z = 0;
  int32_t average_noise_Y = 0;
  int32_t average_noise_Z = 0;
  int32_t noise_counter = 0;
  uint16_t sit_up_counter_Y = 0;
  uint16_t sit_up_counter_Z = 0;
  int16_t stddev_Z = 4;
  int16_t stddev_Y = 2;
  int16_t event_probability_Z = 100;
  int16_t event_probability_Y = 100;
  uint8_t state_Y = 0;
  uint8_t state_Z = 0;
  warpPrint("Beginning calibration \n Stay still");
  while (OSA_TimeGetMsec() - start_time < 1000) {
    // this is calibration were the user should stay still to estimate the noise
    // which can then be used for calibration
    int16_t Y = getSensorDataMMA8451Q_Y_axis();
    int16_t Z = getSensorDataMMA8451Q_Z_axis();
    if (Y > max_noise_Y) {
      max_noise_Y = Y;
    } else if (Y < min_noise_Y) {
      min_noise_Y = Y;
    }
    if (Z > max_noise_Z) {
      max_noise_Z = Z;
    } else if (Z < min_noise_Z) {
      min_noise_Z = Z;
    }
    average_noise_Y = average_noise_Y + Y;
    average_noise_Z = average_noise_Z + Z;
    noise_counter++;
    OSA_TimeDelay(10);
    warpPrint(".");
  }
  warpPrint("Ready!");
  average_noise_Y = average_noise_Y / noise_counter;
  average_noise_Z = average_noise_Z / noise_counter;
  warpPrint("%d,%d,\n", average_noise_Y, average_noise_Z);
  max_noise_Y = (max_noise_Y - average_noise_Y) * 2;
  min_noise_Y = (min_noise_Y - average_noise_Y) * 2;
  max_noise_Z = (max_noise_Z - average_noise_Z) * 2;
  min_noise_Z = (min_noise_Z - average_noise_Z) * 2;
  int16_t previous_value_Y = getSensorDataMMA8451Q_Y_axis() - average_noise_Y;
  int32_t end_calibration = OSA_TimeGetMsec();

  int16_t previous_value_Z = getSensorDataMMA8451Q_Z_axis() - average_noise_Z;

  // do rolling average
  while (OSA_TimeGetMsec() - end_calibration < 20000) { // record for 20 seconds
    sum_Z = 0;
    sum_Y = 0;

    for (uint16_t i = 0; i < size; i++) {
      // only care about the Y and Z axis accelerations
      //
      int16_t Y = getSensorDataMMA8451Q_Y_axis();
      int16_t Z = getSensorDataMMA8451Q_Z_axis();

      // want to add all these values to an array to be average over
      sum_Y = sum_Y + Y - average_noise_Y;
      sum_Z = sum_Z + Z - average_noise_Z;
    }
    int16_t average_Y = sum_Y / size;
    int16_t average_Z = sum_Z / size;
    warpPrint("%d, %d\n", average_Z, average_Y); // this is the rolling
                                                 // average at this value
    // Can use the highest and lowest values to determine
    // confidence.
    if ((average_Y > highest_Y) && (state_Y == 1)) {
      highest_Y = average_Y;
    } else if ((average_Y < lowest_Y) && (state_Y == 0)) {
      lowest_Y = average_Y;
    }
    // now want to work out if there is a crossing over the gravity threshold
    int16_t up_threshold = -2000;

    int16_t down_threshold = -900;
    if ((state_Y == 0) && (average_Y < up_threshold)) {
      // reached peak from lying down
      // change to state 1
      state_Y = 1;
    }
    if ((state_Y == 1) && (average_Y > down_threshold)) {
      // going back down
      state_Y = 2;
    }
    if ((state_Y == 2) && (previous_value_Y > average_Y)) {
      int16_t difference_down_Y = highest_Y - down_threshold;
      int16_t difference_up_Y = up_threshold - lowest_Y;
      int16_t probability_of_correct_Y = 0;
      // working out the probability of the noise causing the accelerations to
      // be tipped over the threshold
      probability_of_correct_Y =
          probability(difference_down_Y, difference_up_Y, stddev_Y);
      sit_up_counter_Y++;
      event_probability_Y = event_probability_Y * probability_of_correct_Y;
      event_probability_Y = event_probability_Y / 100;
      // warpPrint("Sit up_Y!");
      highest_Y = 0;
      lowest_Y = 0;
      state_Y = 0;
    }
    previous_value_Y = average_Y;

    // Can use the highest and lowest values to determine
    // confidence.
    if ((average_Z > highest_Z) && (state_Z == 1)) {
      highest_Z = average_Z;
    } else if ((average_Z < lowest_Z) && (state_Z == 0)) {
      lowest_Z = average_Z;
    }
    // now want to work out if there is a crossing over the gravity threshold
    int16_t up_threshold_Z = -1800;

    int16_t down_threshold_Z = -600;
    if ((state_Z == 0) && (average_Z < up_threshold_Z)) {
      // reached peak from lying down
      // change to state 1
      state_Z = 1;
    }
    if ((state_Z == 1) && (average_Z > down_threshold_Z)) {
      // going back down
      state_Z = 2;
    }
    if ((state_Z == 2) && (previous_value_Z > average_Z)) {
      // this means have reached lowest value
      // want to assess the probablilty of this event occuring
      // see if noise could have tipped the value over the threshold by
      // comparing the highest and lowest values to the thresholds
      int16_t difference_down_Z = highest_Z - down_threshold_Z;
      int16_t difference_up_Z = up_threshold_Z - lowest_Z;
      // warpPrint("differences %d,%d\n", difference_up_Z, difference_down_Z);
      int16_t probability_of_correct_Z = 0;
      // working out the probability of the noise causing the accelerations to
      // be tipped over the threshold
      probability_of_correct_Z =
          probability(difference_down_Z, difference_up_Z, stddev_Z);
      // warpPrint("%d \n ", probability_of_correct_Z);
      sit_up_counter_Z++;
      event_probability_Z = event_probability_Z * probability_of_correct_Z;
      event_probability_Z = event_probability_Z / 100;
      // warpPrint("Sit up_Z!");
      highest_Z = 0;
      lowest_Z = 0;
      state_Z = 0;
    }

    previous_value_Z = average_Z;
  }

  // now want to asses the probabilities of their being that many sit ups.
  if (sit_up_counter_Z == sit_up_counter_Y) {
    warpPrint("Number of sit ups : %d \n", sit_up_counter_Y);
    int16_t confidence = event_probability_Z * event_probability_Y / 100;
    warpPrint("with %d percent confidence", confidence);
  } else {
    warpPrint("Number of sit ups : %d \n", sit_up_counter_Y);
    int16_t confidence_Y = event_probability_Y / 2;
    warpPrint("with %d percent confidence", confidence_Y);
    warpPrint("Number of sit ups : %d \n", sit_up_counter_Z);
    int16_t confidence_Z = event_probability_Z / 2;
    warpPrint("with %d percent confidence", confidence_Z);
  }
}
