#ifndef _MICO_ROS_MICO_ROS_SENSORS_RANGE_H_
#define _MICO_ROS_MICO_ROS_SENSORS_RANGE_H_

#include "pico/stdlib.h"

namespace micoros {

/**
 * @brief A Range sensor.
 */
struct RangeSensor {

  /**
   * @brief Construct a new Range Sensor object.
   *
   * @param echo_pin The echo GPIO pin.
   * @param trigger_pin The trigger GPIO pin.
   */
  RangeSensor(
      unsigned int echo_pin,
      unsigned int trigger_pin,
      unsigned int trigger_duration = 10
  )
  : echo_pin_(echo_pin)
  , trigger_pin_(trigger_pin)
  , trigger_duration_(trigger_duration) {
    gpio_init(echo_pin_);
    gpio_set_dir(echo_pin_, GPIO_IN);
    gpio_init(trigger_pin_);
    gpio_set_dir(trigger_pin_, GPIO_OUT);

    gpio_put(trigger_pin, 0);
    sleep_us(200);
  }

  /**
   * @brief Get the range value in meter.
   *
   * @return float the range reading in meter.
   */
  float read() const {

    // Send an impulse trigger
    gpio_put(trigger_pin_, 1);
    sleep_us(trigger_duration_);
    gpio_put(trigger_pin_, 0);

    // Read how long is the echo
    uint32_t signaloff, signalon;
    do {
      signaloff = time_us_32();
    } while (gpio_get(echo_pin_) == 0);

    do {
      signalon = time_us_32();
    } while (gpio_get(echo_pin_) == 1);

    // Actual echo duration in us
    float dt = signalon - signaloff;

    // distance in meter:
    // echo duration (us) x speed of sound (m/us) / 2 (round trip)
    return dt * 0.000343 / 2.0;
  }

protected:
  const unsigned int echo_pin_;
  const unsigned int trigger_pin_;
  const unsigned int trigger_duration_ = 10;
};

} // namespace micoros

#endif // _MICO_ROS_MICO_ROS_SENSORS_RANGE_H_


