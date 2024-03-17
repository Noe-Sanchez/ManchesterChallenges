#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>


rcl_subscription_t subscription;
rcl_publisher_t publisher;
rcl_publisher_t voltage_publisher;
std_msgs_msg_Float32 voltage_msg;
std_msgs_msg_Int32 potentiometer_msg;
std_msgs_msg_Float32 pwm_duty_cycle_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;


#define Pin_LED 32
#define Pin_PWM 25

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop() {
 while (1) {
  digitalWrite(Pin_LED, !digitalRead(Pin_LED));
  delay(100);
 }
}

void timer1_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) {
  //read pot
  potentiometer_msg.data = analogRead(33);
 }
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) {
  RCSOFTCHECK(rcl_publish(&voltage_publisher, &voltage_msg, NULL));
  RCSOFTCHECK(rcl_publish(&publisher, &potentiometer_msg, NULL));
  //publish voltage
  float raw_v = float(analogRead(33));
  voltage_msg.data = map(raw_v, 0, 4096, 0, 3.3 * 100) / 100.0;
 }
}

void pwm_generator_callback(const void * duty_cycle)
{
 //subscribe to PWM generator
 const std_msgs_msg_Float32 * duty_cycle_msg = (const std_msgs_msg_Float32 *)duty_cycle;
 int pwm_value = (duty_cycle_msg->data) * 2.55;
 ledcWrite(0, pwm_value);
}

void setup() {
 set_microros_transports();

 pinMode(Pin_PWM, OUTPUT);
 pinMode(Pin_LED, OUTPUT);

 digitalWrite(Pin_LED, HIGH);
  
 // Configuration of channel 0 with the chosen frequency and resolution
 ledcSetup(0, 5000, 8);
 ledcAttachPin(Pin_PWM, 0);



 delay(2000);

 allocator = rcl_get_default_allocator();

 //create init_options
 RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

 // create node
 RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32", "", &support));

 // create publisher
 RCCHECK(rclc_publisher_init_default(
      &voltage_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/voltage"));

 RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/raw_pot"));

 // create subscriber
 RCCHECK(rclc_subscription_init_default(
      &subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pwm_duty_cycle"));

 // create timers
 const unsigned int timer_timeout = 1000;
 RCCHECK(rclc_timer_init_default(
      &timer_1,
      &support,
      RCL_MS_TO_NS(10),
      timer1_callback));

 RCCHECK(rclc_timer_init_default(
      &timer_2,
      &support,
      RCL_MS_TO_NS(100),
      timer2_callback));

 // create executor
 RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
 RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
 RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
 RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &pwm_duty_cycle_msg, &pwm_generator_callback, ON_NEW_DATA));

 potentiometer_msg.data = 0.0;
 voltage_msg.data = 0;
 pwm_duty_cycle_msg.data = 0;
}

void loop() {
 delay(100);
 RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

