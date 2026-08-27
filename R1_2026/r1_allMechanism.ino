#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <NativeEthernet.h>
#include <Encoder.h>
#include <IntervalTimer.h>
#include <ServoEasing.h>

// =====================================================
// BNO
// =====================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

float yaw = 0;
float bno_ax = 0;
float bno_ay = 0;
bool bno_ok = false;

// =====================================================
// MOTOR CONFIG
// =====================================================
int pwmR_pin[3] = {3, 5, 13};
int pwmL_pin[3] = {2, 4, 12};

Encoder m[3] = {
  Encoder(21, 20),
  Encoder(27, 26),
  Encoder(40, 41)

};

float cpr[] = { 3550.0, 3550.0, 3550.0 };

// =====================================================
// ARM
// =====================================================
Encoder armEnc(32, 33);

int armRPWM = 15;
int armLPWM = 14;

float armKP = 16.0;
float armKI = 0.9;
float armKD = 1.0;

float armErr = 0;
float armPrevErr = 0;
float armInteg = 0;
float armPID = 0;

long armSP_ticks = 0;

// ---- ARM LIMITS ----
const float TICKS_PER_DEG = 17500.0 / 360.0;                // ~48.6
const long ARM_MIN_TICKS = (long)(-270.0 * TICKS_PER_DEG);  // 0
const long ARM_MAX_TICKS = (long)(270.0 * TICKS_PER_DEG);   // ~4375

// =====================================================
// SERVOS
// =====================================================
ServoEasing kfswrist;
ServoEasing kfsgrip;
ServoEasing staffwrist;
ServoEasing staffgrip;
ServoEasing arucoservo;
int lastStaffWristTarget = 0;


// =====================================================
// Conveyor
// =====================================================
int conveyorSpeed = 12000;

int convRPWM = 23;
int convLPWM = 22;

// =====================================================
// VALVES
// =====================================================
int valve1_pin = 100;
int valve2_pin = 200;

// =====================================================
// RPM PID
// =====================================================
volatile float rpm_sp[3] = { 0, 0, 0 };
volatile float rpm_rt[3] = { 0, 0, 0 };

float kp[] = { 9.0, 9.0, 9.0 };
float ki[] = { 165.0, 165.0, 165.0 };
float kd[] = { 0.5, 0.5, 0.5 };

float error[3] = { 0, 0, 0 };
float eInt[3] = { 0, 0, 0 };
float eDer[3] = { 0, 0, 0 };
float lastError[3] = { 0, 0, 0 };

volatile long oldPosition[3] = { 0, 0, 0 };
volatile long newPosition[3] = { 0, 0, 0 };
volatile long count[3] = { 0, 0, 0 };

volatile int pwm_pid[3] = { 0, 0, 0 };

float rpm_cmd[3];

// =====================================================
// MICRO ROS
// =====================================================
rcl_subscription_t rpm_subscriber;
rcl_subscription_t actuator_subscriber;

rcl_publisher_t bno_publisher;
rcl_publisher_t rpm_feedback_publisher;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

std_msgs__msg__Float32MultiArray rpm_msg;
std_msgs__msg__Float32MultiArray actuator_msg;
std_msgs__msg__Float32 imu_msg;
std_msgs__msg__Float32MultiArray rpm_feedback_msg;

#define RPM_CMD_SIZE 3
#define ACTUATOR_SIZE 10
#define RPM_FEEDBACK_SIZE 3

float rpm_cmd_data[RPM_CMD_SIZE];
float actuator_data[ACTUATOR_SIZE];
float rpm_feedback_data[RPM_FEEDBACK_SIZE];

// =====================================================
// TIMERS
// =====================================================
IntervalTimer motorTimer;

// =====================================================
// MACROS
// =====================================================
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

// =====================================================
// ERROR LOOP
// =====================================================
void error_loop() {
  while (1) { delay(100); }
}

// =====================================================
// MOTOR DRIVER
// =====================================================
void runMotor(int pwm_val, int pwmLPin, int pwmRPin) {
  analogWrite(pwmRPin, pwm_val <= 0 ? -pwm_val : 0);
  analogWrite(pwmLPin, pwm_val >= 0 ? pwm_val : 0);
}

// =====================================================
// MESSAGE INIT
// =====================================================
void setup_messages() {
  rpm_msg.data.data = rpm_cmd_data;
  rpm_msg.data.capacity = RPM_CMD_SIZE;
  rpm_msg.data.size = RPM_CMD_SIZE;

  actuator_msg.data.data = actuator_data;
  actuator_msg.data.capacity = ACTUATOR_SIZE;
  actuator_msg.data.size = ACTUATOR_SIZE;

  rpm_feedback_msg.data.data = rpm_feedback_data;
  rpm_feedback_msg.data.capacity = RPM_FEEDBACK_SIZE;
  rpm_feedback_msg.data.size = RPM_FEEDBACK_SIZE;
}

// =====================================================
// RPM CALLBACK
// =====================================================
void rpm_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size != 3) return;

  rpm_sp[0] = msg->data.data[0];
  rpm_sp[1] = msg->data.data[1];
  rpm_sp[2] = msg->data.data[2];
  Serial.print(rpm_sp[0]);
  Serial.print(" ");
  Serial.print(rpm_sp[1]);
  Serial.print(" ");
  Serial.print(rpm_sp[2]);
  Serial.println();
}

// =====================================================
// ACTUATOR CALLBACK
// =====================================================
void actuator_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size != 10) return;

  Serial.print("ACTUATOR CMD: ");
  for (int i = 0; i < 10; i++) {
    Serial.print(msg->data.data[i]);
    Serial.print(" ");
  }
  Serial.println();

  // ARM — clamped to tick limits
  float armAngle = constrain(msg->data.data[0], 0.0, 270.0);
  armSP_ticks = (long)(armAngle * TICKS_PER_DEG);

  // SERVOS — clamped to match Python limits
  kfswrist.write(constrain(msg->data.data[1], 40, 140));
  kfsgrip.write(constrain(msg->data.data[2], 20, 125));
  // staffwrist.easeTo(constrain(msg->data.data[3], 20, 100),400);
  staffwrist.write(constrain(msg->data.data[3], 0,90));
  staffgrip.write(constrain(msg->data.data[5], 40,75));
  arucoservo.write(constrain(msg->data.data[4], 0, 180)); 
  int target = constrain(msg->data.data[3], 0, 90);

  // if(target != lastStaffWristTarget)
  // {
  //     staffwrist.easeTo(target, 100);
  //     lastStaffWristTarget = target;
  // }
  // CONVEYOR
  float conveyor = msg->data.data[7];

  if (conveyor > 0.1){
    conveyorSpeed = 12000;
    runMotor(conveyorSpeed, convLPWM, convRPWM);}
  else if (conveyor < -0.1){
    conveyorSpeed = 9000;
    runMotor(-conveyorSpeed, convLPWM, convRPWM);}
  else
    runMotor(0, convLPWM, convRPWM);

  // VALVES
  digitalWrite(valve1_pin, msg->data.data[8] > 0.5 ? HIGH : LOW);
  digitalWrite(valve2_pin, msg->data.data[9] > 0.5 ? HIGH : LOW);
}

// =====================================================
// MOTOR UPDATE (75ms ISR)
// =====================================================
void motor_update() {
  // ARM PID
  long pos = armEnc.read();

  armErr = armSP_ticks - pos;
  armInteg += armErr * 0.075;
  armInteg = constrain(armInteg, -2000, 2000);

  float der1 = (armErr - armPrevErr) / 0.075;

  armPID = (armKP * armErr) + (armKI * armInteg) + (armKD * der1);

  armPID = constrain(armPID, -5000, 5000);

  armPrevErr = armErr;

  runMotor(armPID, armLPWM, armRPWM);

  // RPM FEEDBACK
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();

    count[i] = abs(newPosition[i] - oldPosition[i]);

    rpm_rt[i] = count[i] / cpr[i] * 600 * 4.0 / 3;

    rpm_rt[i] *= (newPosition[i] < oldPosition[i]) ? -1 : 1;

    oldPosition[i] = newPosition[i];
  }

  rpm_cmd[0] = rpm_sp[0];
  rpm_cmd[1] = rpm_sp[1];
  rpm_cmd[2] = rpm_sp[2];

  // Serial.print("RPM CMD: ");
  // Serial.print(rpm_sp[0]);
  // Serial.print(" , ");
  // Serial.print(rpm_sp[1]);
  // Serial.print(" , ");
  // Serial.println(rpm_sp[2]);
  // // RPM PID

  for (int i = 0; i < 3; i++) {
    if(abs(rpm_cmd[i]) < 0.5)
    {
        eInt[i]      = 0.0;
        lastError[i] = 0.0;
        analogWrite(pwmR_pin[i], 0);
        analogWrite(pwmL_pin[i], 0);
        continue;
    }

    error[i] = rpm_cmd[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] += error[i] * 0.075;

    pwm_pid[i] = int(
      kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);

    pwm_pid[i] = constrain(pwm_pid[i], -12000, 12000);

    analogWrite(pwmR_pin[i], pwm_pid[i] >= 0 ? pwm_pid[i] : 0);
    analogWrite(pwmL_pin[i], pwm_pid[i] <= 0 ? -pwm_pid[i] : 0);

    lastError[i] = error[i];
  }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  analogWriteResolution(14);

  Wire1.begin();
  bno_ok = bno.begin();
  if (bno_ok) bno.setExtCrystalUse(true);

  // PINS
  pinMode(armRPWM, OUTPUT);
  pinMode(armLPWM, OUTPUT);
  pinMode(convRPWM, OUTPUT);
  pinMode(convLPWM, OUTPUT);
  pinMode(valve1_pin, OUTPUT);
  pinMode(valve2_pin, OUTPUT);

  digitalWrite(valve1_pin, LOW);
  digitalWrite(valve2_pin, LOW);

  // SERVOS
  kfswrist.attach(8);
  kfsgrip.attach(9);
  staffwrist.attach(10);
  staffgrip.attach(11);
  kfswrist.write(35);
  kfsgrip.write(90);
  staffwrist.write(80);
  staffgrip.write(40);
  // arucoservo.attach(21);
  arucoservo.write(0);

  // ETHERNET
  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };
  IPAddress arduino_ip(192, 168, 1, 15);
  IPAddress agent_ip(192, 168, 1, 10);

  set_microros_native_ethernet_udp_transports(
    arduino_mac, arduino_ip, agent_ip, 9999);

  delay(2000);
  Serial.println("Setup Start");

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));

  RCCHECK(rclc_support_init_with_options(
    &support,
    0,
    NULL,
    &init_options,
    &allocator));
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  setup_messages();

  // RPM SUBSCRIBER
  RCCHECK(rclc_subscription_init_default(
    &rpm_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_rpm_cmd"));

  // ACTUATOR SUBSCRIBER
  RCCHECK(rclc_subscription_init_default(
    &actuator_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "actuator_cmd"));

  // BNO PUBLISHER
  RCCHECK(rclc_publisher_init_default(
    &bno_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "imu_yaw"));

  // RPM FEEDBACK PUBLISHER
  RCCHECK(rclc_publisher_init_default(
    &rpm_feedback_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "wheel_rpm_feedback"));

  // EXECUTOR
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &rpm_subscriber,
    &rpm_msg, &rpm_callback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &actuator_subscriber,
    &actuator_msg, &actuator_callback, ON_NEW_DATA));

  Serial.println("Setup done");
  // MOTOR TIMER
  motorTimer.begin(motor_update, 75000);
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static unsigned long last_bno = 0;
  static unsigned long last_pub = 0;

  // BNO UPDATE
  if (bno_ok && millis() - last_bno > 10) {
    sensors_event_t accel;
    sensors_event_t euler;

    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&euler, Adafruit_BNO055::VECTOR_EULER);

    bno_ax = accel.acceleration.x * 100.0;
    bno_ay = accel.acceleration.y * 100.0;
    yaw = euler.orientation.x;

    last_bno = millis();
  }


  staffwrist.update();

  // EXECUTOR
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // PUBLISH @ 50Hz
  if (millis() - last_pub > 20) {
    // IMU YAW
    imu_msg.data = yaw;
    RCSOFTCHECK(rcl_publish(&bno_publisher, &imu_msg, NULL));

    // RPM FEEDBACK
    rpm_feedback_data[0] = rpm_rt[0];
    rpm_feedback_data[1] = rpm_rt[1];
    rpm_feedback_data[2] = rpm_rt[2];
    rpm_feedback_msg.data.data = rpm_feedback_data;
    RCSOFTCHECK(rcl_publish(&rpm_feedback_publisher, &rpm_feedback_msg, NULL));

    last_pub = millis();
  }
}