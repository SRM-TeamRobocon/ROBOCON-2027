// Mecanum drive PID + odometry

#include <micro_ros_arduino.h>
#include <NativeEthernet.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Encoder.h>

#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MAX_RPM 250
#define NUM_MOTORS 4

float rpm_cmd[4];


// Motor pins: FL, FR, RL, RR
int drive_omni[4][2] = {
  {6, 7},
  {4, 5},
  {0, 1},
  {2, 3}
};

Encoder m[4] = {
  Encoder(34, 35),
  Encoder(16, 17),
  Encoder(41, 40),
  Encoder(26, 27)
};

float dt = 0.075;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;

rcl_publisher_t odom_publisher;
std_msgs__msg__Float32MultiArray odom_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

const int FORWARD_SIGN = 1;   // dead wheel that measures robot-forward
const int STRAFE_SIGN  = 1;   // dead wheel that measures robot-left
const int HEADING_SIGN = 1;  // BNO055 is clockwise-positive, so -1 gives CCW-positive

double headingOffset = 0.0;   // makes the start pose 0 deg

IntervalTimer motorTimer;

volatile float motor_pwm[NUM_MOTORS] = {0, 0, 0, 0};

volatile int pwm_pid[] = {0, 0, 0, 0};
volatile float rpm_sp[] = {0, 0, 0, 0};

volatile float kp[] = {09.0, 9.0, 09.0, 09.0};
volatile float ki[] = {165.0, 165.0, 165.0, 165.0};
volatile float kd[] = {00.50, 00.5, 00.50, 00.50};

float error[] = {0, 0, 0, 0};
float eInt[] = {0, 0, 0, 0};
float eDer[] = {0, 0, 0, 0};
float lastError[] = {0, 0, 0, 0};

volatile long oldPosition[4] = {0, 0, 0, 0};
volatile long count[4] = {0, 0, 0, 0};
volatile long newPosition[4] = {0, 0, 0, 0};

volatile float rpm_rt[4] = {0, 0, 0, 0};

float cpr[] = {3500.0, 3500.0, 3500.0, 3500.0};

// yaw correction
#define YAW_TOL 2.0          // degrees
#define YAW_DEADBAND 0.5     // degrees

float rotate_kp = 2.0;

volatile float currentYawDegrees = 0.0;

// Odometry
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Encoder forwardWheelEncoder(21, 20);
Encoder strafeWheelEncoder(33, 32);

const double ENCODER_CPR = 2400.0;
const double DEAD_WHEEL_DIAMETER_MM = 57.8;
const double DEAD_WHEEL_CIRCUMFERENCE_MM = PI * DEAD_WHEEL_DIAMETER_MM;
const double MM_PER_ENCODER_TICK = DEAD_WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR;

const double FORWARD_WHEEL_OFFSET_MM = 33.0;
const double STRAFE_WHEEL_OFFSET_MM = 235.0;

double robotFieldX = 0.0;
double robotFieldY = 0.0;
double robotHeadingRadians = 0.0;

long previousForwardWheelTicks = 0;
long previousStrafeWheelTicks = 0;
double previousHeadingRadians = 0.0;
bool odometryInitialized = false;

unsigned long lastOdomPublishTime = 0;
const unsigned long ODOM_PUBLISH_INTERVAL = 50;

float odom_data[3];

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

// Motor drive
void drive(int pwmL, int pwmR, int pwm) {
  analogWrite(pwmL, pwm < 0 ? -pwm : 0);
  analogWrite(pwmR, pwm > 0 ? pwm : 0);
} 

void updateImu() {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

  currentYawDegrees = event.orientation.x;

  // Convert 0–360 to -180–180
  if (currentYawDegrees > 180.0) {
    currentYawDegrees -= 360.0;
  }
}

float getYawCorrection() {
  float yaw = currentYawDegrees; // 0 to 359

  // Convert 0–360 to -180–180
  if (yaw > 180.0) {
    yaw -= 360.0;
  }

  // Small error = exactly zero
  if (abs(yaw) < YAW_DEADBAND) {
    yaw = 0;
  }

  float w = 0;

  if (abs(yaw) > YAW_TOL) {
    w = -rotate_kp * yaw;
    w = constrain(w, -80, 80);
  } else {
    w = 0;
  }

  return w;
}

// Motor PID update
void motor_update() {
  // for (int i = 0; i < 4; i++) {
  //   newPosition[i] = m[i].read();
  //   ::count[i] = abs(newPosition[i] - oldPosition[i]);
  //   rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
  //   rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
  //   ::count[i] = 0;
  //   oldPosition[i] = newPosition[i];
  // }
  

  // float rpm_cmd[4];
  float w = getYawCorrection();
  Serial.println(w);

  // Original commands
  float fl = motor_pwm[0];
  float fr = motor_pwm[1];
  float rl = motor_pwm[2];
  float rr = motor_pwm[3];

  // float fl = 0;
  // float fr = 0;
  // float rl = 0;
  // float rr = 0;

  // Add yaw correction
  fl += w;
  fr += w;
  rl += w;
  rr += w;

  // Limit commands
  fl = constrain(fl, -MAX_RPM, MAX_RPM);
  fr = constrain(fr, -MAX_RPM, MAX_RPM);
  rl = constrain(rl, -MAX_RPM, MAX_RPM);
  rr = constrain(rr, -MAX_RPM, MAX_RPM);

  rpm_cmd[0] = fl;
  rpm_cmd[1] = fr;
  rpm_cmd[2] = rl;
  rpm_cmd[3] = rr;

  // for (int i = 0; i < 4; i++) {
  //   rpm_cmd[i] = motor_pwm[i];
  // }

  for (int i = 0; i < 4; i++) {
    // error[i] = rpm_cmd[i] - rpm_rt[i];
    // eDer[i] = (error[i] - lastError[i]) / dt;
    // eInt[i] = eInt[i] + error[i] * dt;

    // pwm_pid[i] = int(kp[i] * error[i] + 0 * eInt[i] + kd[i] * eDer[i]);

    // pwm_pid[i] = pwm_pid[i] % 16383;
    // float output = kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i];

    // pwm_pid[i] = constrain(pwm_pid[i], -16383.0f, 16383.0f);

    // pwm_pid[i] = (int)output;

    rpm_cmd[i] = map(rpm_cmd[i], -250, 250, -13000, 13000);

    drive(drive_omni[i][0], drive_omni[i][1], (int)rpm_cmd[i]);

    // lastError[i] = error[i];

  }
  // for (int i=0; i<=3; i++){
    // Serial.printf("RPM_output(motor-%d:%0.2f\n", i + 1, rpm_cmd[i]);
    // Serial.printf("RPM_%d_input:%0.2f\n", i + 1, rpm_cmd[i]);
  // }
}

// Normalize angle
double normalizeAngle(double angle) {
  if (angle > PI) angle -= 2.0 * PI;
  if (angle <= -PI) angle += 2.0 * PI;
  return angle;
}

// Read BNO055 heading
double readImuHeading() {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  double raw = event.orientation.x * PI / 180.0;
  return normalizeAngle(HEADING_SIGN * (raw - headingOffset));
}

// Update odometry
void updateOdometry() {
  long currentForward = FORWARD_SIGN * forwardWheelEncoder.read();
  long currentStrafe  = STRAFE_SIGN  * strafeWheelEncoder.read();

  if (!odometryInitialized) {
    // capture the starting heading so the start pose is 0 deg
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    headingOffset = event.orientation.x * PI / 180.0;

    previousForwardWheelTicks = currentForward;
    previousStrafeWheelTicks = currentStrafe;
    previousHeadingRadians = 0.0;
    robotHeadingRadians = 0.0;
    odometryInitialized = true;
    return;
  }

  double currentHeading = readImuHeading();
  robotHeadingRadians = currentHeading;

  double dForward = (currentForward - previousForwardWheelTicks) * MM_PER_ENCODER_TICK;
  double dStrafe  = (currentStrafe  - previousStrafeWheelTicks)  * MM_PER_ENCODER_TICK;
  double dHeading = normalizeAngle(currentHeading - previousHeadingRadians);

  double robotForward  = dForward - FORWARD_WHEEL_OFFSET_MM * dHeading;
  double robotSideways = dStrafe  - STRAFE_WHEEL_OFFSET_MM  * dHeading;

  double avgHeading = previousHeadingRadians + dHeading / 2.0;

  robotFieldX += robotForward * cos(avgHeading) - robotSideways * sin(avgHeading);
  robotFieldY += robotForward * sin(avgHeading) + robotSideways * cos(avgHeading);

  previousForwardWheelTicks = currentForward;
  previousStrafeWheelTicks = currentStrafe;
  previousHeadingRadians = currentHeading;
}


// ROS callback
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *incoming = (const std_msgs__msg__Float32MultiArray *)msgin;

  if (incoming->data.size < NUM_MOTORS) {
    return;
  }

  motor_pwm[0] = constrain(incoming->data.data[0], -MAX_RPM, MAX_RPM);
  motor_pwm[1] = constrain(incoming->data.data[1], -MAX_RPM, MAX_RPM);
  motor_pwm[2] = constrain(incoming->data.data[2], -MAX_RPM, MAX_RPM);
  motor_pwm[3] = constrain(incoming->data.data[3], -MAX_RPM, MAX_RPM);

  // for (int i=0; i<=3; i++){
  //   Serial.printf("motor_pwm(motor-%d:%0.2f\n", i + 1, pwm_pid[i]);
  //   // Serial.printf("RPM_%d_input:%0.2f\n", i + 1, rpm_cmd[i]);
  // }


}

// Teensy MAC
#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
  for (uint8_t by = 0; by < 2; by++)
    mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;

  for (uint8_t by = 0; by < 4; by++)
    mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.print("hi");

  analogWriteResolution(14);

  // Initialize BNO055
  Wire1.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
  } else {
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 initialized.");
  }

  byte arduino_mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF};

#if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
#endif

  IPAddress arduino_ip(192, 168, 1, 150);
  IPAddress agent_ip(192, 168, 1, 100);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(drive_omni[i][0], OUTPUT);
    pinMode(drive_omni[i][1], OUTPUT);

    analogWrite(drive_omni[i][0], 0);
    analogWrite(drive_omni[i][1], 0);
  }

  set_microros_native_ethernet_udp_transports(arduino_mac, arduino_ip, agent_ip, 9999);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "teensy_align", "", &support));

  // Original target RPM subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/target_rpm"
  ));

  // Odometry publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/teensy_odom"
  ));

  std_msgs__msg__Float32MultiArray__init(&msg);

  msg.data.data = (float *)malloc(NUM_MOTORS * sizeof(float));
  msg.data.size = NUM_MOTORS;
  msg.data.capacity = NUM_MOTORS;

  odom_msg.data.data = odom_data;
  odom_msg.data.size = 3;
  odom_msg.data.capacity = 3;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA
  ));

  motorTimer.begin(motor_update, 75000);
}

void readPIDTuning() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    float newKp, newKi, newKd;
    int motorno;

    if (sscanf(command.c_str(), "%d %f %f %f", &motorno, &newKp, &newKi, &newKd) == 4) {
      kp[motorno] = newKp;
      ki[motorno] = newKi;
      kd[motorno] = newKd;

      for (int i = 0; i < NUM_MOTORS; i++) {
        eInt[i] = 0;
        lastError[i] = 0;
      }
    }
  }
}

void loop() {

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  updateImu();
  // for (int i=0; i<=3; i++){
  //   Serial.printf("RPM_output(motor-%d:%0.2f\n", i + 1, eInt[i]);
  //   // Serial.printf("RPM_%d_input:%0.2f\n", i + 1, rpm_cmd[i]);
  // }
  unsigned long now = millis();

  if (now - lastOdomPublishTime >= ODOM_PUBLISH_INTERVAL) {
    updateOdometry();

    odom_msg.data.data[0] = (float)robotFieldX;
    odom_msg.data.data[1] = (float)robotFieldY;
    odom_msg.data.data[2] = (float)(robotHeadingRadians * 180.0 / PI);

    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

    lastOdomPublishTime = now;
  }
}

// ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999
