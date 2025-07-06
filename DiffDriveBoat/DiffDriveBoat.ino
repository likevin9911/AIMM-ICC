#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// --- Pin definitions ---
#define LED_PIN            14  // On-board status LED
#define LEFT_MOTOR_PIN     12  // PWM output to Smart Duo Drive 30 left channel
#define RIGHT_MOTOR_PIN    13  // PWM output to Smart Duo Drive 30 right channel

// --- Calibration (measured) ---
constexpr int LEFT_PWM_NEUTRAL   = 1495;
constexpr int RIGHT_PWM_NEUTRAL  = 1496;
constexpr int LEFT_PWM_MAX       = 2006;
constexpr int LEFT_PWM_MIN       =  990;
constexpr int RIGHT_PWM_MAX      = 2006;
constexpr int RIGHT_PWM_MIN      =  990;

// Deadband and slew-rate limits
constexpr float DEADBAND        = 0.05f;  // 5% around neutral
constexpr float SLEW_RATE       = 0.02f;  // max change per control loop
constexpr unsigned long TIMEOUT = 500;    // ms to stop if no cmd_vel
constexpr unsigned long SPIN_INTERVAL = 10; // ms between executor spins

// Globals for ROS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Servo objects
Servo leftMotor;
Servo rightMotor;

// State variables
static float current_left_speed  = 0.0f;
static float current_right_speed = 0.0f;
static unsigned long last_msg_time = 0;
static unsigned long last_spin = 0;

#define RCCHECK(fn)    { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn){ rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) error_loop(); }

void error_loop() {
  // Blink LED on error
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Map a normalized speed [-1.0,1.0] to PWM range
int mapPWM(float speed, int neutral, int p_max, int p_min) {
  if (speed >= 0) {
    return neutral + int(speed * (p_max - neutral));
  } else {
    return neutral + int(speed * (neutral - p_min));
  }
}

void subscription_callback(const void* msgin) {
  const auto* twist = (const geometry_msgs__msg__Twist*)msgin;
  float lin = twist->linear.x;
  float ang = twist->angular.z;

  // Differential drive mixing
  float left_speed  = lin - ang;
  float right_speed = lin + ang;

  // Constrain and apply deadband
  left_speed  = constrain(left_speed,  -1.0f, 1.0f);
  right_speed = constrain(right_speed, -1.0f, 1.0f);
  if (fabs(left_speed)  < DEADBAND)  left_speed  = 0;
  if (fabs(right_speed) < DEADBAND)  right_speed = 0;

  // Slew-rate limit
  auto slew = [](float target, float &current) {
    float delta = target - current;
    if (delta >  SLEW_RATE) delta =  SLEW_RATE;
    if (delta < -SLEW_RATE) delta = -SLEW_RATE;
    current += delta;
  };
  slew(left_speed,  current_left_speed);
  slew(right_speed, current_right_speed);

  // Compute PWM
  int left_pwm  = mapPWM(current_left_speed,  LEFT_PWM_NEUTRAL,  LEFT_PWM_MAX,  LEFT_PWM_MIN);
  int right_pwm = mapPWM(current_right_speed, RIGHT_PWM_NEUTRAL, RIGHT_PWM_MAX, RIGHT_PWM_MIN);

  // Send to motors
  leftMotor.writeMicroseconds(left_pwm);
  rightMotor.writeMicroseconds(right_pwm);

  // Status LED: ON when moving
  digitalWrite(LED_PIN, (current_left_speed != 0 || current_right_speed != 0));

  // Track heartbeat
  last_msg_time = millis();
}

void setup() {
  // USB-CDC transport for micro-ROS
  set_microros_transports();

  // Pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.writeMicroseconds(LEFT_PWM_NEUTRAL);
  rightMotor.writeMicroseconds(RIGHT_PWM_NEUTRAL);

  delay(2000);  // Wait for agent

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  ));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA
  ));

  last_msg_time = millis();
  last_spin = millis();
}

void loop() {
  unsigned long now = millis();

  // Spin executor at SPIN_INTERVAL
  if (now - last_spin >= SPIN_INTERVAL) {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(SPIN_INTERVAL)));
    last_spin = now;
  }

  // Failsafe: stop motors if no cmd_vel
  if (now - last_msg_time > TIMEOUT) {
    leftMotor.writeMicroseconds(LEFT_PWM_NEUTRAL);
    rightMotor.writeMicroseconds(RIGHT_PWM_NEUTRAL);
    digitalWrite(LED_PIN, LOW);
    current_left_speed = current_right_speed = 0;
  }
}
