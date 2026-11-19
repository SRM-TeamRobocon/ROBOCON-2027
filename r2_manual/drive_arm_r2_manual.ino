// Combined 4-Wheel Omni Drive + 2-DOF Arm PID Control (micro-ROS)

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <CrashReport.h>
#include <NativeEthernet.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <Encoder.h>
#include <IntervalTimer.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <Servo.h>

// ================= NETWORK CONFIG =================
IPAddress teensy_ip(192, 168, 1, 150);
IPAddress agent_ip(192, 168, 1, 100);
const uint16_t AGENT_PORT = 8888;   // Run: ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999

// ================= DRIVE PARAMETERS & PINS =================
#define NUM_MOTORS 4
#define TUNING_MSG_LEN 20

const float MAX_DRIVE_RPM = 700.0f;
const float MAX_DRIVE_PWM = 13000.0f;

// Drive motor pins: FL, FR, RL, RR
const int drive_omni[NUM_MOTORS][2] = {
  {4, 5},
  {3, 2},
  {7, 6},
  {1, 0}
};

volatile float target_rpm[NUM_MOTORS] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile bool motor_enabled[NUM_MOTORS] = {true, true, true, true};

// ================= SERVO PARAMETERS & PINS =================

const int SERVO1_PIN = 10;
const int SERVO2_PIN = 29;

Servo servo1;
Servo servo2;

volatile int servo1_deg = 90;
volatile int servo2_deg = 90;

// ================= ARM PARAMETERS & PINS =================
const float ARM_DT = 0.075f;        // Must match armTimer period (75 ms)
const float MAX_ARM_PWM = 5000.0f;  // 14-bit resolution -> max 16383

// Arm Cytron driver pins
const int M1_RPWM = 22;   // Shoulder
const int M1_LPWM = 23;
const int M2_RPWM = 18;   // Elbow
const int M2_LPWM = 19;

// Arm encoders
Encoder motor1Enc(12, 11);  // Shoulder
Encoder motor2Enc(41, 26);  // Elbow

volatile long currentTicks1 = 0;
volatile long currentTicks2 = 0;

// Arm PID gains
float KP1 = 23.0f;
float KI1 = 0.0f;
float KD1 = 0.0f;

float KP2 = 25.0f;
float KI2 = 0.0f;
float KD2 = 0.0f;

// Arm PID state & setpoints (ISR-shared)
volatile float m1Err = 0.0f, m1PrevErr = 0.0f, m1Integ = 0.0f;
volatile float m2Err = 0.0f, m2PrevErr = 0.0f, m2Integ = 0.0f;

volatile long m1SP_ticks = 0;
volatile long m2SP_ticks = 0;

const float TPD1 = 2400.0f / 360.0f;
const float TPD2 = 2400.0f / 360.0f;

// ================= MICRO-ROS OBJECTS =================
rcl_subscription_t sub_drive;
std_msgs__msg__Float32MultiArray msg_drive;
float tuning_data[TUNING_MSG_LEN];

rcl_subscription_t sub_arm;
std_msgs__msg__Int32MultiArray msg_arm;
int32_t arm_data[4];

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

IntervalTimer driveTimer;
IntervalTimer armTimer;

#define RCCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      while (1) \
        ; \
    } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) {} \
  }

// ================= DRIVE MOTOR HELPERS & ISR =================
void driveWheel(int pwmL, int pwmR, int pwm) {
  pwm = constrain(pwm, -16383, 16383);
  analogWrite(pwmL, pwm < 0 ? -pwm : 0);
  analogWrite(pwmR, pwm > 0 ? pwm : 0);
}

void driveControlLoop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    float cmd = motor_enabled[i] ? target_rpm[i] : 0.0f;
    cmd = constrain(cmd, -MAX_DRIVE_RPM, MAX_DRIVE_RPM);

    int pwm_out = (int)((cmd / MAX_DRIVE_RPM) * MAX_DRIVE_PWM);
    driveWheel(drive_omni[i][0], drive_omni[i][1], pwm_out);
  }
}

// ================= SERVO CONTROL =================

void runServos(int angle1, int angle2) {

  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);

  servo1.write(angle1);
  servo2.write(angle2);
}

// ================= ARM MOTOR HELPERS & ISR =================
void runBTS(int RPWM, int LPWM, float val) {
  int pwm = abs((int)val);
  pwm = constrain(pwm, 0, (int)MAX_ARM_PWM);

  if (val > 0.0f) {
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
  } else if (val < 0.0f) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void armControlLoop() {
  long ticks1 = motor1Enc.read();
  long ticks2 = motor2Enc.read();
  currentTicks1 = ticks1;
  currentTicks2 = ticks2;

  // --- Shoulder (M1) ---
  float err1 = (float)(m1SP_ticks - ticks1);
  float integ1 = constrain(m1Integ + (err1 * ARM_DT), -200.0f, 200.0f);
  float deriv1 = (err1 - m1PrevErr) / ARM_DT;
  float out1 = (KP1 * err1) + (KI1 * integ1) + (KD1 * deriv1);
  out1 = constrain(out1, -MAX_ARM_PWM, MAX_ARM_PWM);

  m1Err = err1;
  m1Integ = integ1;
  m1PrevErr = err1;

  // --- Elbow (M2) ---
  float err2 = (float)(m2SP_ticks - ticks2);
  float integ2 = constrain(m2Integ + (err2 * ARM_DT), -200.0f, 200.0f);
  float deriv2 = (err2 - m2PrevErr) / ARM_DT;
  float out2 = (KP2 * err2) + (KI2 * integ2) + (KD2 * deriv2);
  out2 = constrain(out2, -MAX_ARM_PWM, MAX_ARM_PWM);

  m2Err = err2;
  m2Integ = integ2;
  m2PrevErr = err2;

  runBTS(M1_RPWM, M1_LPWM, out1);
  runBTS(M2_RPWM, M2_LPWM, out2);
}

// ================= ROS SUBSCRIPTION CALLBACKS =================
void driveCallback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *incoming =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (incoming->data.size < NUM_MOTORS) {
    return;
  }

  float new_rpm[NUM_MOTORS];
  bool new_en[NUM_MOTORS];

  for (int i = 0; i < NUM_MOTORS; i++) {
    new_rpm[i] = constrain(incoming->data.data[i], -MAX_DRIVE_RPM, MAX_DRIVE_RPM);
    new_en[i] = (incoming->data.size >= 20) ? (incoming->data.data[16 + i] > 0.5f) : true;
  }

  noInterrupts();
  for (int i = 0; i < NUM_MOTORS; i++) {
    target_rpm[i] = new_rpm[i];
    motor_enabled[i] = new_en[i];
  }
  interrupts();
}

// void armCallback(const void *msgin) {
//   const std_msgs__msg__Int32MultiArray *msg =
//     (const std_msgs__msg__Int32MultiArray *)msgin;

//   if (msg->data.size < 2) {
//     return;
//   }

//   float t1_deg = msg->data.data[0];
//   float t2_deg = msg->data.data[1];



//   long new_sp1 = (long)(t1_deg * TPD1);
//   long new_sp2 = (long)(t2_deg * TPD2);

//   noInterrupts();
//   m1SP_ticks = new_sp1;
//   m2SP_ticks = new_sp2;
//   m1Integ = 0.0f;
//   m1PrevErr = 0.0f;
//   m2Integ = 0.0f;
//   m2PrevErr = 0.0f;
//   interrupts();
// }

void armCallback(const void *msgin) {

  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  // Expecting 4 values
  if (msg->data.size < 4) {
    return;
  }

  // DC Motor angles
  float t1_deg = msg->data.data[0];
  float t2_deg = msg->data.data[1];

  // Servo angles
  int new_servo1_deg = msg->data.data[2];
  int new_servo2_deg = msg->data.data[3];

  // Convert DC motor angles to encoder ticks
  long new_sp1 = (long)(t1_deg * TPD1);
  long new_sp2 = (long)(t2_deg * TPD2);

  // Update setpoints
  noInterrupts();

  m1SP_ticks = new_sp1;
  m2SP_ticks = new_sp2;

  m1Integ = 0.0f;
  m1PrevErr = 0.0f;

  m2Integ = 0.0f;
  m2PrevErr = 0.0f;

  servo1_deg = constrain(new_servo1_deg, 0, 180);
  servo2_deg = constrain(new_servo2_deg, 0, 180);

  interrupts();

  // Run servos
  runServos(servo1_deg, servo2_deg);
}

// ================= TEENSY MAC =================
#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
  for (uint8_t by = 0; by < 2; by++)
    mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;

  for (uint8_t by = 0; by < 4; by++)
    mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}
#endif

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  if (CrashReport) {
    Serial.print(CrashReport);
  }

  analogWriteResolution(14);

  // Initialize drive motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(drive_omni[i][0], OUTPUT);
    pinMode(drive_omni[i][1], OUTPUT);
    analogWrite(drive_omni[i][0], 0);
    analogWrite(drive_omni[i][1], 0);
  }

  // Initialize arm motor pins
  pinMode(M1_RPWM, OUTPUT);
  pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT);
  pinMode(M2_LPWM, OUTPUT);
  analogWrite(M1_RPWM, 0);
  analogWrite(M1_LPWM, 0);
  analogWrite(M2_RPWM, 0);
  analogWrite(M2_LPWM, 0);

  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Set initial servo positions
  servo1.write(90); //wrist
  servo2.write(120);

  // Start hardware control timers
  driveTimer.begin(driveControlLoop, 20000);  // 50 Hz drive update
  armTimer.begin(armControlLoop, 75000);      // 75 ms arm PID loop (matches ARM_DT)

  byte arduino_mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF};
#if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
#endif

  set_microros_native_ethernet_udp_transports(arduino_mac, teensy_ip, agent_ip, AGENT_PORT);

  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    Serial.println("Waiting for micro-ROS agent...");
    delay(500);
  }
  Serial.println("Connected to micro-ROS agent!");

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_robot", "", &support));

  // 1. Drive subscription (tuning_commands)
  RCCHECK(rclc_subscription_init_default(
    &sub_drive,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "tuning_commands"
  ));

  std_msgs__msg__Float32MultiArray__init(&msg_drive);
  msg_drive.data.data = tuning_data;
  msg_drive.data.size = 0;
  msg_drive.data.capacity = TUNING_MSG_LEN;

  // 2. Arm subscription (arm_angles)
  RCCHECK(rclc_subscription_init_default(
    &sub_arm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "arm_angles"
  ));

  std_msgs__msg__Int32MultiArray__init(&msg_arm);
  msg_arm.data.data = arm_data;
  msg_arm.data.size = 0;
  msg_arm.data.capacity = 4;

  // Executor with 2 subscription handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));


  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &sub_drive,
    &msg_drive,
    &driveCallback,
    ON_NEW_DATA
  ));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &sub_arm,
    &msg_arm,
    &armCallback,
    ON_NEW_DATA
  ));

  Serial.println("Teensy node started: 4-wheel drive + 2-DOF arm");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}