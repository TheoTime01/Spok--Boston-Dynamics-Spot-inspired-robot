#ifndef _MICO_ROS_MICO_ROS_MACRO_H_
#define _MICO_ROS_MICO_ROS_MACRO_H_

#define RCCHECK(fn) {                                                           \
  rcl_ret_t temp_rc = fn;                                                       \
  if((temp_rc != RCL_RET_OK)){                                                  \
    printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);  \
    return 1;                                                                   \
  }}

#define RCSOFTCHECK(fn) {                                                         \
  rcl_ret_t temp_rc = fn;                                                         \
  if((temp_rc != RCL_RET_OK)){                                                    \
    printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);  \
  }}

#endif // _MICO_ROS_MICO_ROS_MACRO_H_


