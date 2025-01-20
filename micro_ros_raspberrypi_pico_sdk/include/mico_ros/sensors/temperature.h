#ifndef _MICO_ROS_MICO_ROS_TEMPERATURE_H_
#define _MICO_ROS_MICO_ROS_TEMPERATURE_H_

#include "mico_ros/pico_utils.h"

#include "hardware/adc.h"
#include <limits>

namespace micoros {

enum class TEMPERATURE_UNITS : char {
  Celcius,
  Farenheit
};

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c
 */
template <TEMPERATURE_UNITS U = TEMPERATURE_UNITS::Celcius>
float read_onboard_temperature() {

  // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
  constexpr float conversion_factor = 3.3f / (1 << 12);

  float adc = adc_read() * conversion_factor;
  float temp_celcius = 27.0 - (adc - 0.706) / 0.001721;

  if constexpr (U == TEMPERATURE_UNITS::Celcius) {
    return temp_celcius;
  } else if (U == TEMPERATURE_UNITS::Farenheit) {
    return temp_celcius * 9 / 5 + 32;
  }

  return std::numeric_limits<float>::signaling_NaN();
}

void init_onboard_temperature() {
  // Initialize hardware AD converter, enable onboard temperature sensor and
  // select its channel (do this once for efficiency, but beware that this
  // is a global operation).
  adc_init();
  adc_set_temp_sensor_enabled(true);
  adc_select_input(TEMPERATURE_SENSOR_INPUT);
}

} // namespace micoros

#endif // _MICO_ROS_MICO_ROS_TEMPERATURE_H_


