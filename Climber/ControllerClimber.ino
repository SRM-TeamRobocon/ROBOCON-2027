#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include <Encoder.h>
#include <IntervalTimer.h>

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// ================================================================
// ETHERNET
// ================================================================

byte mac[] = {
  0x04,
  0xE9,
  0xE5,
  0x00,
  0x00,
  0x04
};

// ================================================================
// RACK AND PINION
// ================================================================

const int RP1[2] = {23, 22};   // RPWM, LPWM
const int RP2[2] = {13, 12};   // RPWM, LPWM

const int encRP1[2] = {29, 28};
const int encRP2[2] = {30, 31};

Encoder enc1(encRP1[0], encRP1[1]);
Encoder enc2(encRP2[0], encRP2[1]);

// ================================================================
// LIMIT SWITCHES
// ================================================================

const int RP1_LIMIT_PIN = 9;
const int RP2_LIMIT_PIN = 8;

// LOW = switch pressed
bool rp1_limit_active = false;
bool rp2_limit_active = false;

// ================================================================
// 4 TRACTION MOTORS
// ================================================================

// M1, M2 = front
// M3, M4 = rear

const int traction[4][2] = {
  {3, 2},      // M1: Forward, Reverse
  {0, 1},      // M2: Forward, Reverse
  {36, 37},    // M3: Forward, Reverse
  {4, 5}       // M4: Forward, Reverse
};

// ================================================================
// RACK PID
// ================================================================

const int PWM_RESOLUTION = 14;

const int BENCH_MANUAL_PWM = 3000;
const int BENCH_HOLD_MAX_PWM = 3000;

float Kp = 3.0;
float Ki = 0.0;
float Kd = 0.05;

long targetTicks[2] = {0, 0};

long previousError[2] = {0, 0};

float integral[2] = {0, 0};

// ================================================================
// PID TIMER
// ================================================================

IntervalTimer pidTimer;

volatile bool pidRun = false;

void pidTimerISR()
{
  pidRun = true;
}

// ================================================================
// RACK MODES
// ================================================================

enum RackMode
{
  RACK_HOLD,
  RACK_FORWARD,
  RACK_REVERSE
};

RackMode rackMode[2] = {
  RACK_HOLD,
  RACK_HOLD
};

// ================================================================
// MICRO-ROS
// ================================================================

rcl_subscription_t sub_joy_cmd;

rcl_publisher_t pub_limit_switch;

std_msgs__msg__Float32MultiArray msg_joy_cmd;
std_msgs__msg__Float32MultiArray msg_limit_switch;

rclc_executor_t executor;

rclc_support_t support;

rcl_allocator_t allocator;

rcl_node_t node;

// ================================================================
// INCOMING MESSAGE
//
// [RP1 command, RP2 command, LEFT traction, RIGHT traction]
// ================================================================

float joy_cmd_data[4] = {
  0,
  0,
  0,
  0
};

// ================================================================
// LIMIT SWITCH MESSAGE
//
// [RP1 limit, RP2 limit]
// ================================================================

float limit_switch_data[2] = {
  0,
  0
};

// ================================================================
// COMMAND VARIABLES
// ================================================================

volatile float rp1_cmd = 0.0f;

volatile float rp2_cmd = 0.0f;

// LEFT TRACTION = M1 + M3
volatile float traction_left_cmd = 0.0f;

// RIGHT TRACTION = M2 + M4
volatile float traction_right_cmd = 0.0f;

// ================================================================
// DEBUG
// ================================================================

unsigned long lastDebugPrint = 0;

// ================================================================
// LIMIT SWITCH PUBLISH TIMER
// ================================================================

unsigned long lastLimitPublish = 0;

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);

  // --------------------------------------------------------------
  // Rack motors
  // --------------------------------------------------------------

  pinMode(RP1[0], OUTPUT);
  pinMode(RP1[1], OUTPUT);

  pinMode(RP2[0], OUTPUT);
  pinMode(RP2[1], OUTPUT);

  // --------------------------------------------------------------
  // Limit switches
  // --------------------------------------------------------------

  pinMode(RP1_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RP2_LIMIT_PIN, INPUT_PULLUP);

  // --------------------------------------------------------------
  // Traction motors
  // --------------------------------------------------------------

  for (int i = 0; i < 4; i++)
  {
    pinMode(traction[i][0], OUTPUT);
    pinMode(traction[i][1], OUTPUT);
  }

  // --------------------------------------------------------------
  // PWM
  // --------------------------------------------------------------

  analogWriteResolution(PWM_RESOLUTION);

  // --------------------------------------------------------------
  // Stop everything initially
  // --------------------------------------------------------------

  stopMotor();

  stopTraction();

  // --------------------------------------------------------------
  // Initial rack targets
  // --------------------------------------------------------------

  targetTicks[0] = enc1.read();

  targetTicks[1] = enc2.read();

  // --------------------------------------------------------------
  // PID timer
  //
  // 10000 us = 10 ms = 100 Hz
  // --------------------------------------------------------------

  pidTimer.begin(pidTimerISR, 10000);

  // ==============================================================
  // ETHERNET
  // ==============================================================

  IPAddress client_ip(
    192, 168, 1, 50
  );

  IPAddress agent_ip(
    192, 168, 1, 100
  );

  uint16_t agent_port = 8888;

  Ethernet.begin(
    mac,
    client_ip
  );

  delay(2000);

  Serial.print("Teensy IP: ");
  Serial.println(Ethernet.localIP());

  // ==============================================================
  // MICRO-ROS ETHERNET TRANSPORT
  // ==============================================================

  set_microros_native_ethernet_udp_transports(
    mac,
    client_ip,
    agent_ip,
    agent_port
  );

  delay(2000);

  // ==============================================================
  // MICRO-ROS INIT
  // ==============================================================

  allocator = rcl_get_default_allocator();

  rclc_support_init(
    &support,
    0,
    NULL,
    &allocator
  );

  rclc_node_init_default(
    &node,
    "teensy_rack_controller",
    "",
    &support
  );

  // ==============================================================
  // /rpm_sp SUBSCRIBER
  // ==============================================================

  rclc_subscription_init_default(
    &sub_joy_cmd,
    &node,

    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs,
      msg,
      Float32MultiArray
    ),

    "/rpm_sp"
  );

  // ==============================================================
  // /limit_switch PUBLISHER
  // ==============================================================

  rclc_publisher_init_default(
    &pub_limit_switch,
    &node,

    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs,
      msg,
      Float32MultiArray
    ),

    "/limit_switch"
  );

  // ==============================================================
  // MESSAGE MEMORY
  // ==============================================================

  msg_joy_cmd.data.data = joy_cmd_data;
  msg_joy_cmd.data.size = 0;
  msg_joy_cmd.data.capacity = 4;

  msg_limit_switch.data.data = limit_switch_data;
  msg_limit_switch.data.size = 2;
  msg_limit_switch.data.capacity = 2;

  // ==============================================================
  // EXECUTOR
  // ==============================================================

  rclc_executor_init(
    &executor,
    &support.context,
    1,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_joy_cmd,
    &msg_joy_cmd,
    &joyCmdCallback,
    ON_NEW_DATA
  );

  Serial.println(
    "=================================================="
  );

  Serial.println(
    "Teensy Rack & Pinion Controller Started"
  );

  Serial.println(
    "Limit switches: RP1=32, RP2=33"
  );

  Serial.println(
    "=================================================="
  );
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
  // --------------------------------------------------------------
  // Process micro-ROS
  // --------------------------------------------------------------

  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(1)
  );

  // --------------------------------------------------------------
  // LIMIT SWITCH
  // --------------------------------------------------------------

  processLimitSwitches();

  // --------------------------------------------------------------
  // Rack motors
  // --------------------------------------------------------------

  runRackMotors();

  // --------------------------------------------------------------
  // PID
  // --------------------------------------------------------------

  if (pidRun)
  {
    pidRun = false;

    positionPID();
  }

  // --------------------------------------------------------------
  // Traction
  // --------------------------------------------------------------

  runTraction();

  // --------------------------------------------------------------
  // Publish limit switch status
  // --------------------------------------------------------------

  if (millis() - lastLimitPublish >= 20)
  {
    lastLimitPublish = millis();

    publishLimitSwitches();
  }

  // --------------------------------------------------------------
  // Debug
  // --------------------------------------------------------------

  if (millis() - lastDebugPrint > 200)
  {
    lastDebugPrint = millis();

    Serial.print("RP1: ");
    Serial.print(enc1.read());

    Serial.print(" / ");
    Serial.print(targetTicks[0]);

    Serial.print("    RP2: ");
    Serial.print(enc2.read());

    Serial.print(" / ");
    Serial.print(targetTicks[1]);

    Serial.print("    CMD: ");

    Serial.print(rp1_cmd);
    Serial.print(", ");

    Serial.print(rp2_cmd);

    Serial.print("    LIMIT: ");

    Serial.print(rp1_limit_active);
    Serial.print(", ");

    Serial.print(rp2_limit_active);

    Serial.print("    LEFT=");
    Serial.print(traction_left_cmd);

    Serial.print("    RIGHT=");
    Serial.println(traction_right_cmd);
  }
}

// ================================================================
// MICRO-ROS CALLBACK
// ================================================================

void joyCmdCallback(
  const void *msgin
)
{
  const std_msgs__msg__Float32MultiArray *msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size >= 4)
  {
    // ------------------------------------------------------------
    // RACK COMMANDS
    // ------------------------------------------------------------

    rp1_cmd =
      msg->data.data[0];

    rp2_cmd =
      msg->data.data[1];

    // ------------------------------------------------------------
    // TRACTION COMMANDS
    //
    // data[2] = LEFT
    // data[3] = RIGHT
    // ------------------------------------------------------------

    traction_left_cmd =
      msg->data.data[2];

    traction_right_cmd =
      msg->data.data[3];
  }
}

// ================================================================
// LIMIT SWITCH PROCESSING
//
// Direction 1 = positive command
// Direction 2 = negative command
//
// If the limit is active:
//   Direction 1 -> blocked / O equivalent
//   Direction 2 -> allowed
// ================================================================

void processLimitSwitches()
{
  rp1_limit_active =
    (digitalRead(RP1_LIMIT_PIN) == LOW);

  rp2_limit_active =
    (digitalRead(RP2_LIMIT_PIN) == LOW);

  // --------------------------------------------------------------
  // RP1
  // --------------------------------------------------------------

  if (rp1_limit_active && rp1_cmd > 0.5f)
  {
    // Same command as CIRCLE/O
    rp1_cmd = 0.0f;
  }

  // --------------------------------------------------------------
  // RP2
  // --------------------------------------------------------------

  if (rp2_limit_active && rp2_cmd > 0.5f)
  {
    // Same command as CIRCLE/O
    rp2_cmd = 0.0f;
  }
}

// ================================================================
// PUBLISH LIMIT SWITCH STATUS
//
// data[0] = RP1
// data[1] = RP2
//
// 0 = not pressed
// 1 = pressed
// ================================================================

void publishLimitSwitches()
{
  limit_switch_data[0] =
    rp1_limit_active ? 1.0f : 0.0f;

  limit_switch_data[1] =
    rp2_limit_active ? 1.0f : 0.0f;

  msg_limit_switch.data.size = 2;

  rcl_publish(
    &pub_limit_switch,
    &msg_limit_switch,
    NULL
  );
}

// ================================================================
// RACK MOTORS
// ================================================================

void runRackMotors()
{
  // ==============================================================
  // RP1
  // ==============================================================

  if (rp1_cmd > 0.5f)
  {
    rackMode[0] = RACK_FORWARD;
  }

  else if (rp1_cmd < -0.5f)
  {
    rackMode[0] = RACK_REVERSE;
  }

  else
  {
    // RP1 is not commanded to move.
    // Capture its CURRENT position and hold with PID.

    targetTicks[0] = enc1.read();

    integral[0] = 0;

    previousError[0] = 0;

    rackMode[0] = RACK_HOLD;
  }

  // ==============================================================
  // RP2
  // ==============================================================

  if (rp2_cmd > 0.5f)
  {
    rackMode[1] = RACK_FORWARD;
  }

  else if (rp2_cmd < -0.5f)
  {
    rackMode[1] = RACK_REVERSE;
  }

  else
  {
    // RP2 is not commanded to move.
    // Capture its CURRENT position and hold with PID.

    targetTicks[1] = enc2.read();

    integral[1] = 0;

    previousError[1] = 0;

    rackMode[1] = RACK_HOLD;
  }

  // ==============================================================
  // RP1 OUTPUT
  // ==============================================================

  if (rackMode[0] == RACK_FORWARD)
  {
    motorForward(
      RP1[0],
      RP1[1],
      BENCH_MANUAL_PWM
    );
  }

  else if (rackMode[0] == RACK_REVERSE)
  {
    motorReverse(
      RP1[0],
      RP1[1],
      BENCH_MANUAL_PWM
    );
  }

  // ==============================================================
  // RP2 OUTPUT
  // ==============================================================

  if (rackMode[1] == RACK_FORWARD)
  {
    motorForward(
      RP2[0],
      RP2[1],
      BENCH_MANUAL_PWM
    );
  }

  else if (rackMode[1] == RACK_REVERSE)
  {
    motorReverse(
      RP2[0],
      RP2[1],
      BENCH_MANUAL_PWM
    );
  }
}

// ================================================================
// POSITION PID
// ================================================================

void positionPID()
{
  const float dt = 0.01;

  Encoder* encoders[2] = {
    &enc1,
    &enc2
  };

  const int* motors[2] = {
    RP1,
    RP2
  };

  for (int i = 0; i < 2; i++)
  {
    if (rackMode[i] != RACK_HOLD)
    {
      continue;
    }

    long currentTicks =
      encoders[i]->read();

    long error =
      targetTicks[i] - currentTicks;

    float P =
      Kp * error;

    integral[i] +=
      error * dt;

    float I =
      Ki * integral[i];

    float D =
      Kd *
      (error - previousError[i])
      / dt;

    previousError[i] =
      error;

    float output =
      P + I + D;

    output =
      constrain(
        output,
        -BENCH_HOLD_MAX_PWM,
        BENCH_HOLD_MAX_PWM
      );

    if (output > 0)
    {
      motorForward(
        motors[i][0],
        motors[i][1],
        (int)output
      );
    }

    else if (output < 0)
    {
      motorReverse(
        motors[i][0],
        motors[i][1],
        (int)-output
      );
    }

    else
    {
      analogWrite(
        motors[i][0],
        0
      );

      analogWrite(
        motors[i][1],
        0
      );
    }
  }
}

// ================================================================
// TRACTION
//
// data[2] = LEFT
// data[3] = RIGHT
//
// LEFT  = M1 + M3
// RIGHT = M2 + M4
//
// EXISTING LOGIC UNCHANGED
// ================================================================

void runTraction()
{
  int left_pwm =
    (int)(traction_left_cmd * 64.0f);

  int right_pwm =
    (int)(traction_right_cmd * 64.0f);

  left_pwm = constrain(
    left_pwm,
    -8192,
    8192
  );

  right_pwm = constrain(
    right_pwm,
    -8192,
    8192
  );

  // LEFT SIDE

  driveMotor(
    traction[0][0],
    traction[0][1],
    right_pwm
  );

  driveMotor(
    traction[2][0],
    traction[2][1],
    left_pwm
  );

  // RIGHT SIDE

  driveMotor(
    traction[1][0],
    traction[1][1],
    left_pwm
  );

  driveMotor(
    traction[3][0],
    traction[3][1],
    right_pwm
  );
}

// ================================================================
// TRACTION MOTOR
// ================================================================

void driveMotor(
  int pinF,
  int pinR,
  int pwm
)
{
  if (pwm > 0)
  {
    analogWrite(
      pinF,
      0
    );

    analogWrite(
      pinR,
      pwm
    );
  }

  else if (pwm < 0)
  {
    analogWrite(
      pinF,
      -pwm
    );

    analogWrite(
      pinR,
      0
    );
  }

  else
  {
    analogWrite(
      pinF,
      0
    );

    analogWrite(
      pinR,
      0
    );
  }
}

// ================================================================
// RACK FORWARD
// ================================================================

void motorForward(
  int rpwm,
  int lpwm,
  int pwm
)
{
  analogWrite(
    lpwm,
    0
  );

  analogWrite(
    rpwm,
    pwm
  );
}

// ================================================================
// RACK REVERSE
// ================================================================

void motorReverse(
  int rpwm,
  int lpwm,
  int pwm
)
{
  analogWrite(
    lpwm,
    pwm
  );

  analogWrite(
    rpwm,
    0
  );
}

// ================================================================
// STOP RACK MOTORS
// ================================================================

void stopMotor()
{
  analogWrite(
    RP1[0],
    0
  );

  analogWrite(
    RP1[1],
    0
  );

  analogWrite(
    RP2[0],
    0
  );

  analogWrite(
    RP2[1],
    0
  );
}

// ================================================================
// STOP TRACTION
// ================================================================

void stopTraction()
{
  for (int i = 0; i < 4; i++)
  {
    analogWrite(
      traction[i][0],
      0
    );

    analogWrite(
      traction[i][1],
      0
    );
  }
}