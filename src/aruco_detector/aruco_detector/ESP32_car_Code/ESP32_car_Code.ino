#include <micro_ros_arduino.h>

#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LEFT_PWM 25
#define LEFT_IN1 26
#define LEFT_IN2 27

#define RIGHT_PWM 33
#define RIGHT_IN1 14
#define RIGHT_IN2 12

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define LEFT_CHANNEL 0
#define RIGHT_CHANNEL 1

float max_speed = 200;

void setMotor(int pwm_pin, int in1, int in2, int speed)
{
  if(speed > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(speed < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  speed = constrain(speed,0,255);
  ledcWrite(pwm_pin,speed);
}

void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  float left = linear - angular;
  float right = linear + angular;

  int left_speed = left * max_speed;
  int right_speed = right * max_speed;

  setMotor(LEFT_CHANNEL, LEFT_IN1, LEFT_IN2, left_speed);
  setMotor(RIGHT_CHANNEL, RIGHT_IN1, RIGHT_IN2, right_speed);
}

void setup()
{
  set_microros_wifi_transports("YOUR_WIFI",
                               "YOUR_PASSWORD",
                               "192.168.1.100",
                               8888);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  ledcSetup(LEFT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM, LEFT_CHANNEL);

  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_PWM, RIGHT_CHANNEL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_car_node", "", &support);

  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  rclc_executor_add_subscription(
      &executor,
      &subscriber,
      &msg,
      &cmd_vel_callback,
      ON_NEW_DATA);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}