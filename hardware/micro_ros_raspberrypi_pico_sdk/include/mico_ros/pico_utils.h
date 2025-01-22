#ifndef _MICO_ROS_MICO_ROS_UTILS_H_
#define _MICO_ROS_MICO_ROS_UTILS_H_

#include "pico/stdlib.h"

namespace micoros {

#define LED_PIN PICO_DEFAULT_LED_PIN

// See Pi Pico C/C++ SDK 4.1.1. hardware_adc
#define TEMPERATURE_SENSOR_INPUT 4

// 9 characters + null terminating
constexpr char PICO_FRAME[10] = "pico_link";

// 17 characters + null terminating
constexpr char SONAR_0_FRAME[18] = "pico_sonar_0_link";
constexpr char SONAR_1_FRAME[18] = "pico_sonar_1_link";
constexpr char SONAR_2_FRAME[18] = "pico_sonar_2_link";
constexpr char SONAR_3_FRAME[18] = "pico_sonar_3_link";

// Max reading frequency for the sonar.
// Based on the hc-sr04 module datasheet,
// the max echo is 36ms if there is no obstacle,
// to which one adds one trigger duration 10ms.
// 1/0.046=21.7Hz, rounded to 20Hz.
#define SONAR_FREQ 20 // Hz

// #define I2C_PORT i2c0
#define I2C_PORT i2c_default // i2c1
#define I2C_FREQ 400 * 1000

// constexpr unsigned int SDA_PIN = 1;
// constexpr unsigned int SCL_PIN = 2;

#define SDA_PIN PICO_DEFAULT_I2C_SDA_PIN // 4
#define SCL_PIN PICO_DEFAULT_I2C_SCL_PIN // 5

} // namespace micoros

#endif // _MICO_ROS_MICO_ROS_UTILS_H_