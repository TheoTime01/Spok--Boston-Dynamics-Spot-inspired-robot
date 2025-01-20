#ifndef _MICO_ROS_MICO_ROS_ROS_UTILS_H_
#define _MICO_ROS_MICO_ROS_ROS_UTILS_H_

#include <rmw_uros/options.h>
#include <time.h>

extern "C" {
  #include "mico_ros/pico_uart_transports.h"
}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

#include <std_msgs/msg/header.h>

namespace micoros {

/**
 * @brief Fill the timestamp message using the current onboard time.
 *
 * @tparam Msg
 * @param msg The input message to fill.
 */
inline void fill_msg_stamp(builtin_interfaces__msg__Time& msg) {

  timespec tv = {0};
  clock_gettime(0, &tv);

  msg.sec = tv.tv_sec;
  msg.nanosec = tv.tv_nsec;
}

} // namespace micoros

#endif // _MICO_ROS_MICO_ROS_ROS_UTILS_H_