#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

const uint LED_PIN = 25;
const uint LED_1 = 4;
const uint LED_2 = 5;
const uint LED_3 = 6;
const uint LED_4 = 7;

const uint ECHO_PIN_1 = 9;
const uint TRIGGER_PIN_1 = 10;
const uint ECHO_PIN_2 = 11;
const uint TRIGGER_PIN_2 = 12;


rcl_publisher_t publisher;
rcl_publisher_t publisher_bis;
rcl_subscription_t subscriber;

std_msgs__msg__Int32 msg_sub;
std_msgs__msg__Float32 msg_pub;
std_msgs__msg__Float32 msg_pub_bis;

int state = 0;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{

    gpio_put(TRIGGER_PIN_1, 1);
    sleep_us(10);
    gpio_put(TRIGGER_PIN_1, 0);

    uint32_t signaloff, signalon;
    do {
      signaloff = time_us_32();
    } while (gpio_get(ECHO_PIN_1) == 0);

    do {
      signalon = time_us_32();
    } while (gpio_get(ECHO_PIN_1) == 1);

    float dt = signalon - signaloff;
    msg_pub.data =  dt * 0.000343 / 2.0;

    rcl_ret_t ret = rcl_publish(&publisher, &msg_pub, NULL);

    gpio_put(TRIGGER_PIN_2, 1);
    sleep_us(10);
    gpio_put(TRIGGER_PIN_2, 0);

    do {
      signaloff = time_us_32();
    } while (gpio_get(ECHO_PIN_2) == 0);

    do {
      signalon = time_us_32();
    } while (gpio_get(ECHO_PIN_2) == 1);

    dt = signalon - signaloff;
    msg_pub_bis.data =  dt * 0.000343 / 2.0;

    ret = rcl_publish(&publisher_bis, &msg_pub_bis, NULL);
    
}

void subscription_callback(const void * msgin)
{   
    if(state == 0){
*        state = 1;
    }else{
        state = 0;
    }
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LED_1);
    gpio_set_dir(LED_1, GPIO_OUT);
    gpio_init(LED_2);
    gpio_set_dir(LED_2, GPIO_OUT);
    gpio_init(LED_3);
    gpio_set_dir(LED_3, GPIO_OUT);
    gpio_init(LED_4);
    gpio_set_dir(LED_4, GPIO_OUT);

    gpio_init(ECHO_PIN_1);
    gpio_set_dir(ECHO_PIN_1, GPIO_IN);
    gpio_init(TRIGGER_PIN_1);
    gpio_set_dir(TRIGGER_PIN_1, GPIO_OUT);
    gpio_init(ECHO_PIN_2);
    gpio_set_dir(ECHO_PIN_2, GPIO_IN);
    gpio_init(TRIGGER_PIN_2);
    gpio_set_dir(TRIGGER_PIN_2, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pico_publisher");
    rclc_publisher_init_default(
        &publisher_bis,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pico_publisher_bis");

    rcl_ret_t rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_subscriber");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);    

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    rc = rclc_executor_add_subscription(
        &executor, &subscriber, &msg_sub,
        &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    gpio_put(LED_2, 1);

    gpio_put(TRIGGER_PIN_1, 0);
    gpio_put(TRIGGER_PIN_2, 0);

    msg_pub.data = 0.0;
    msg_pub_bis.data = 0.0;
    msg_sub.data = 0;

    while (true)
    {
        
        if(state == 1){
            gpio_put(LED_PIN, 1);
            gpio_put(LED_1, 1);
            gpio_put(LED_2, 1);
            gpio_put(LED_3, 0);
            gpio_put(LED_4, 0);
            sleep_ms(500);
    
            gpio_put(LED_PIN, 0);
            gpio_put(LED_1, 0);
            gpio_put(LED_2, 0);
            gpio_put(LED_3, 1);
            gpio_put(LED_4, 1);
            sleep_ms(500);
        }else{
            gpio_put(LED_PIN, 1);
            gpio_put(LED_1, 1);
            gpio_put(LED_2, 1);
            gpio_put(LED_3, 1);
            gpio_put(LED_4, 1);
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
