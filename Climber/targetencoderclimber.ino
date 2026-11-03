#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float32.h>

#include <Encoder.h>
#include <IntervalTimer.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <math.h>

// ================================================================
// ETHERNET
// ================================================================

byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x00, 0x04};

// ================================================================
// RACK AND PINION
// ================================================================

const int RP1[2] = {23, 22};
const int RP2[2] = {13, 12};

const int encRP1[2] = {21, 20}; 
const int encRP2[2] = {29, 28}; 

Encoder enc1(encRP1[0], encRP1[1]);
Encoder enc2(encRP2[0], encRP2[1]);

// ================================================================
// LIMIT SWITCHES
// ================================================================

const int RP1_LIMIT_PIN = 9;
const int RP2_LIMIT_PIN = 8;

bool rp1_limit_active = false;
bool rp2_limit_active = false;

// ================================================================
// TRACTION MOTORS
// ================================================================

const int traction[4][2] = {
  {3, 2},
  {0, 1},
  {37, 36},
  {5, 4}
};

// ================================================================
// RACK PID AND TARGETS
// ================================================================

const int PWM_RESOLUTION = 14;

const int BENCH_MANUAL_PWM = 8000;
const int BENCH_HOLD_MAX_PWM = 8000;

float Kp = 3.0;
float Ki = 0.0;
float Kd = 0.05;

long targetTicks[2] = {0, 0};

long rackStartPosition[2] = {0, 0};
long rackClimbPosition[2] = {0, 0};

bool rackClimbPositionValid = false;
bool rackPositionCommandActive[2] = {false, false};

// Individual rack travel targets
float rackTargetTicks[2] = {
  3146.0f,
  3146.0f
};

// Individual rack stopping thresholds
float rackThresholdTicks[2] = {
  200.0f,
  200.0f
};

// Target and threshold captured when movement starts
long relativeStopTravelTicks[2] = {2946, 2946};

bool relativeMoveActive[2] = {false, false};
bool relativeHoldActive[2] = {false, false};
bool relativeTargetCompleted[2] = {false, false};

long relativeStartTicks[2] = {0, 0};

long previousError[2] = {0, 0};
float integral[2] = {0, 0};

// ================================================================
// RACK POSITION PUBLISHING
// ================================================================

float rackPositionData[5] = {0, 0, 0, 0, 0};

unsigned long lastRackPositionPublish = 0;
bool previousCaptureClimbRequest = false;

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
// ENCODER PUBLISHING
// ================================================================

long encoderTicks[2] = {0, 0};
float encoderData[2] = {0, 0};

unsigned long lastEncoderPublish = 0;

const int ENCODER_COUNT = 2;

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

rcl_publisher_t pub_encoder_ticks;
rcl_publisher_t pub_rack_positions;
rcl_publisher_t pub_limit_switch;

std_msgs__msg__Float32MultiArray msg_encoder_ticks;
std_msgs__msg__Float32MultiArray msg_rack_positions;
std_msgs__msg__Float32MultiArray msg_joy_cmd;
std_msgs__msg__Float32MultiArray msg_limit_switch;

std_msgs__msg__Float32 msg_rp1_target;
std_msgs__msg__Float32 msg_rp2_target;

std_msgs__msg__Float32 msg_rp1_threshold;
std_msgs__msg__Float32 msg_rp2_threshold;

rcl_subscription_t sub_joy_cmd;

rcl_subscription_t sub_rp1_target;
rcl_subscription_t sub_rp2_target;

rcl_subscription_t sub_rp1_threshold;
rcl_subscription_t sub_rp2_threshold;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// [RP1, RP2, LEFT TRACTION, RIGHT TRACTION, CAPTURE REQUEST]
float joy_cmd_data[5] = {0, 0, 0, 0, 0};

float limit_switch_data[2] = {0, 0};

volatile float rp1_cmd = 0.0f;
volatile float rp2_cmd = 0.0f;

volatile float traction_left_cmd = 0.0f;
volatile float traction_right_cmd = 0.0f;

unsigned long lastDebugPrint = 0;
unsigned long lastLimitPublish = 0;

// ================================================================
// FUNCTION DECLARATIONS
// ================================================================

void joyCmdCallback(const void *msgin);

void rp1TargetCallback(const void *msgin);
void rp2TargetCallback(const void *msgin);

void rp1ThresholdCallback(const void *msgin);
void rp2ThresholdCallback(const void *msgin);

void publishEncoderTicks();
void publishRackPositions();
void publishLimitSwitches();

void readEncoderTicks();
void processLimitSwitches();
void runRackMotors();
void positionPID();
void runTraction();

void driveMotor(int pinF, int pinR, int pwm);
void motorForward(int rpwm, int lpwm, int pwm);
void motorReverse(int rpwm, int lpwm, int pwm);

void stopMotor();
void stopTraction();

// ================================================================
// SETUP
// ================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(RP1[0], OUTPUT);
  pinMode(RP1[1], OUTPUT);

  pinMode(RP2[0], OUTPUT);
  pinMode(RP2[1], OUTPUT);

  pinMode(RP1_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RP2_LIMIT_PIN, INPUT_PULLUP);

  for (int i = 0; i < 4; i++)
  {
    pinMode(traction[i][0], OUTPUT);
    pinMode(traction[i][1], OUTPUT);
  }

  analogWriteResolution(PWM_RESOLUTION);

  stopMotor();
  stopTraction();

  targetTicks[0] = enc1.read();
  targetTicks[1] = enc2.read();

  rackStartPosition[0] = targetTicks[0];
  rackStartPosition[1] = targetTicks[1];

  Serial.print("START POSITION RP1: ");
  Serial.println(rackStartPosition[0]);

  Serial.print("START POSITION RP2: ");
  Serial.println(rackStartPosition[1]);

  pidTimer.begin(pidTimerISR, 10000);

  IPAddress client_ip(192, 168, 1, 50);
  IPAddress agent_ip(192, 168, 1, 100);

  uint16_t agent_port = 8888;

  Ethernet.begin(mac, client_ip);
  delay(2000);

  Serial.print("Teensy IP: ");
  Serial.println(Ethernet.localIP());

  set_microros_native_ethernet_udp_transports(
    mac,
    client_ip,
    agent_ip,
    agent_port
  );

  delay(2000);

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
  // PUBLISHERS
  // ==============================================================

  // /encoder_ticks publisher

  rclc_publisher_init_default(
    &pub_encoder_ticks,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32MultiArray
    ),
    "/encoder_ticks"
  );

  // /rack_positions publisher

  rclc_publisher_init_default(
    &pub_rack_positions,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32MultiArray
    ),
    "/rack_positions"
  );

  // /limit_switch publisher

  rclc_publisher_init_default(
    &pub_limit_switch,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32MultiArray
    ),
    "/limit_switch"
  );

  // ==============================================================
  // SUBSCRIBERS
  // ==============================================================

  // /rpm_sp subscriber

  rclc_subscription_init_default(
    &sub_joy_cmd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32MultiArray
    ),
    "/rpm_sp"
  );

  // RP1 target subscriber

  rclc_subscription_init_default(
    &sub_rp1_target,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32
    ),
    "/rp1_target_ticks"
  );

  // RP2 target subscriber

  rclc_subscription_init_default(
    &sub_rp2_target,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32
    ),
    "/rp2_target_ticks"
  );

  // RP1 threshold subscriber

  rclc_subscription_init_default(
    &sub_rp1_threshold,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32
    ),
    "/rp1_threshold"
  );

  // RP2 threshold subscriber

  rclc_subscription_init_default(
    &sub_rp2_threshold,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      std_msgs, msg, Float32
    ),
    "/rp2_threshold"
  );

  // ==============================================================
  // MESSAGE MEMORY
  // ==============================================================

  msg_joy_cmd.data.data = joy_cmd_data;
  msg_joy_cmd.data.size = 0;
  msg_joy_cmd.data.capacity = 5;

  msg_limit_switch.data.data = limit_switch_data;
  msg_limit_switch.data.size = 2;
  msg_limit_switch.data.capacity = 2;

  msg_encoder_ticks.data.data = encoderData;
  msg_encoder_ticks.data.size = ENCODER_COUNT;
  msg_encoder_ticks.data.capacity = ENCODER_COUNT;

  msg_rack_positions.data.data = rackPositionData;
  msg_rack_positions.data.size = 5;
  msg_rack_positions.data.capacity = 5;

  // ==============================================================
  // EXECUTOR
  // ==============================================================

  // Five subscriptions:
  // 1. /rpm_sp
  // 2. /rp1_target_ticks
  // 3. /rp2_target_ticks
  // 4. /rp1_threshold
  // 5. /rp2_threshold

  rclc_executor_init(
    &executor,
    &support.context,
    5,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_joy_cmd,
    &msg_joy_cmd,
    &joyCmdCallback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_rp1_target,
    &msg_rp1_target,
    &rp1TargetCallback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_rp2_target,
    &msg_rp2_target,
    &rp2TargetCallback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_rp1_threshold,
    &msg_rp1_threshold,
    &rp1ThresholdCallback,
    ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor,
    &sub_rp2_threshold,
    &msg_rp2_threshold,
    &rp2ThresholdCallback,
    ON_NEW_DATA
  );

  Serial.println("========================================");
  Serial.println("Teensy Rack & Pinion Controller Started");
  Serial.println("Individual rack targets and thresholds enabled");
  Serial.println("RP1 Target: 3146 | Threshold: 200");
  Serial.println("RP2 Target: 3146 | Threshold: 200");
  Serial.println("========================================");
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop()
{
  rclc_executor_spin_some(
    &executor,
    RCL_MS_TO_NS(1)
  );

  processLimitSwitches();
  runRackMotors();

  if (pidRun)
  {
    pidRun = false;
    positionPID();
  }

  runTraction();

  if (millis() - lastLimitPublish >= 20)
  {
    lastLimitPublish = millis();
    publishLimitSwitches();
  }

  if (millis() - lastEncoderPublish >= 20)
  {
    lastEncoderPublish = millis();
    publishEncoderTicks();
  }

  if (millis() - lastRackPositionPublish >= 100)
  {
    lastRackPositionPublish = millis();
    publishRackPositions();
  }

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
// ENCODER FUNCTIONS
// ================================================================

void readEncoderTicks()
{
  encoderTicks[0] = enc1.read();
  encoderTicks[1] = enc2.read();
}

void publishEncoderTicks()
{
  readEncoderTicks();

  encoderData[0] = (float)encoderTicks[0];
  encoderData[1] = (float)encoderTicks[1];

  rcl_publish(
    &pub_encoder_ticks,
    &msg_encoder_ticks,
    NULL
  );
}

// ================================================================
// PUBLISH START AND CLIMB POSITIONS
// [START RP1, START RP2, CLIMB RP1, CLIMB RP2, VALID]
// ================================================================

void publishRackPositions()
{
  rackPositionData[0] = (float)rackStartPosition[0];
  rackPositionData[1] = (float)rackStartPosition[1];

  rackPositionData[2] = (float)rackClimbPosition[0];
  rackPositionData[3] = (float)rackClimbPosition[1];

  rackPositionData[4] =
    rackClimbPositionValid ? 1.0f : 0.0f;

  rcl_publish(
    &pub_rack_positions,
    &msg_rack_positions,
    NULL
  );
}

// ================================================================
// RPM COMMAND CALLBACK
// ================================================================

void joyCmdCallback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *msg =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size < 4)
    return;

  float newRP1Command = msg->data.data[0];
  float newRP2Command = msg->data.data[1];

  // Re-arm relative movement only when Jetson sends
  // a command other than -2.

  if (fabsf(newRP1Command + 2.0f) > 0.5f)
    relativeTargetCompleted[0] = false;

  if (fabsf(newRP2Command + 2.0f) > 0.5f)
    relativeTargetCompleted[1] = false;

  rp1_cmd = newRP1Command;
  rp2_cmd = newRP2Command;

  traction_left_cmd = msg->data.data[2];
  traction_right_cmd = msg->data.data[3];

  // Optional fifth element captures climb position.

  bool captureRequest =
    (msg->data.size >= 5 && msg->data.data[4] > 0.5f);

  if (captureRequest && !previousCaptureClimbRequest)
  {
    rackClimbPosition[0] = enc1.read();
    rackClimbPosition[1] = enc2.read();

    rackClimbPositionValid = true;

    Serial.print("CLIMB POSITION STORED: RP1=");
    Serial.print(rackClimbPosition[0]);

    Serial.print(" RP2=");
    Serial.println(rackClimbPosition[1]);
  }

  previousCaptureClimbRequest = captureRequest;
}

// ================================================================
// INDIVIDUAL TARGET CALLBACKS
// ================================================================

void rp1TargetCallback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  rackTargetTicks[0] =
    constrain(fabsf(msg->data), 1.0f, 100000.0f);

  Serial.print("RP1 TARGET UPDATED: ");
  Serial.println(rackTargetTicks[0]);
}


void rp2TargetCallback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  rackTargetTicks[1] =
    constrain(fabsf(msg->data), 1.0f, 100000.0f);

  Serial.print("RP2 TARGET UPDATED: ");
  Serial.println(rackTargetTicks[1]);
}

// ================================================================
// INDIVIDUAL THRESHOLD CALLBACKS
// ================================================================

void rp1ThresholdCallback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  rackThresholdTicks[0] =
    constrain(fabsf(msg->data), 0.0f, 100000.0f);

  Serial.print("RP1 THRESHOLD UPDATED: ");
  Serial.println(rackThresholdTicks[0]);
}


void rp2ThresholdCallback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  rackThresholdTicks[1] =
    constrain(fabsf(msg->data), 0.0f, 100000.0f);

  Serial.print("RP2 THRESHOLD UPDATED: ");
  Serial.println(rackThresholdTicks[1]);
}

// ================================================================
// LIMIT SWITCH PROCESSING
// Direction 1 is blocked when the corresponding switch is pressed.
// ================================================================

void processLimitSwitches()
{
  rp1_limit_active =
    (digitalRead(RP1_LIMIT_PIN) == LOW);

  rp2_limit_active =
    (digitalRead(RP2_LIMIT_PIN) == LOW);

  if (rp1_limit_active && rp1_cmd > 0.5f)
    rp1_cmd = 0.0f;

  if (rp2_limit_active && rp2_cmd > 0.5f)
    rp2_cmd = 0.0f;
}

// ================================================================
// PUBLISH LIMIT SWITCHES
// [RP1, RP2], 0 = released, 1 = pressed
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
//
//  0.0  = stop and hold current position
// +1.0  = continuous direction 1
// -1.0  = continuous direction 2
// +2.0  = move to stored climb position
// -2.0  = relative direction-2 movement
//
// Relative movement stops at:
// requested target - threshold
// ================================================================

void runRackMotors()
{
  Encoder* encoders[2] = {&enc1, &enc2};

  for (int i = 0; i < 2; i++)
  {
    float cmd = (i == 0) ? rp1_cmd : rp2_cmd;

    long current = encoders[i]->read();

    // ------------------------------------------------------------
    // +2.0: move to stored climb position
    // ------------------------------------------------------------

    if (cmd > 1.5f)
    {
      relativeMoveActive[i] = false;
      relativeHoldActive[i] = false;

      if (!rackClimbPositionValid)
      {
        targetTicks[i] = current;

        integral[i] = 0;
        previousError[i] = 0;

        rackPositionCommandActive[i] = false;
        rackMode[i] = RACK_HOLD;
      }
      else
      {
        if (!rackPositionCommandActive[i])
        {
          targetTicks[i] = rackClimbPosition[i];

          integral[i] = 0;
          previousError[i] = 0;
        }

        rackPositionCommandActive[i] = true;
        rackMode[i] = RACK_HOLD;
      }
    }

    // ------------------------------------------------------------
    // -2.0: relative direction-2 movement
    // ------------------------------------------------------------

    else if (cmd < -1.5f)
    {
      rackPositionCommandActive[i] = false;

      // Prevent restarting after target completion.

      if (relativeTargetCompleted[i])
      {
        relativeMoveActive[i] = false;
        rackMode[i] = RACK_HOLD;
      }
      else
      {
        // Capture starting encoder and calculate the stopping
        // travel distance only once when movement begins.

        if (!relativeMoveActive[i])
        {
          relativeHoldActive[i] = false;

          relativeStartTicks[i] = current;

          float requestedTarget = rackTargetTicks[i];
          float threshold = rackThresholdTicks[i];

          float stopTravel =
            requestedTarget - threshold;

          // Prevent zero or negative stopping distance.

          if (stopTravel < 1.0f)
            stopTravel = 1.0f;

          relativeStopTravelTicks[i] =
            (long)stopTravel;

          relativeMoveActive[i] = true;

          integral[i] = 0;
          previousError[i] = 0;

          Serial.print("RP");
          Serial.print(i + 1);

          Serial.print(" RELATIVE MOVE START. Encoder=");
          Serial.print(current);

          Serial.print(" Requested=");
          Serial.print(requestedTarget);

          Serial.print(" Threshold=");
          Serial.print(threshold);

          Serial.print(" Stop after travel=");
          Serial.println(relativeStopTravelTicks[i]);
        }

        // Calculate actual travel from the captured start.

        long travelled =
          labs(current - relativeStartTicks[i]);

        Serial.print("RP");
        Serial.print(i + 1);

        Serial.print(" TRAVELLED=");
        Serial.print(travelled);

        Serial.print(" / STOP AT=");
        Serial.println(relativeStopTravelTicks[i]);

        // --------------------------------------------------------
        // TARGET MINUS THRESHOLD REACHED
        // --------------------------------------------------------

        if (travelled >= relativeStopTravelTicks[i])
        {
          relativeMoveActive[i] = false;
          relativeHoldActive[i] = true;
          relativeTargetCompleted[i] = true;

          rackPositionCommandActive[i] = false;

          // Capture the actual encoder position at stopping.
          // PID will hold this exact position.

          targetTicks[i] = current;

          integral[i] = 0;
          previousError[i] = 0;

          rackMode[i] = RACK_HOLD;

          if (i == 0)
            rp1_cmd = 0.0f;
          else
            rp2_cmd = 0.0f;

          Serial.print("RP");
          Serial.print(i + 1);

          Serial.print(" STOP POSITION=");
          Serial.println(targetTicks[i]);

          Serial.print("RP");
          Serial.print(i + 1);

          Serial.println(" TARGET-THRESHOLD REACHED. PID HOLD.");
        }
        else
        {
          rackMode[i] = RACK_REVERSE;
        }
      }
    }

    // ------------------------------------------------------------
    // +1.0: continuous direction 1
    // ------------------------------------------------------------

    else if (cmd > 0.5f)
    {
      relativeMoveActive[i] = false;
      relativeHoldActive[i] = false;

      rackPositionCommandActive[i] = false;

      rackMode[i] = RACK_FORWARD;
    }

    // ------------------------------------------------------------
    // -1.0: continuous direction 2
    // ------------------------------------------------------------

    else if (cmd < -0.5f)
    {
      relativeMoveActive[i] = false;
      relativeHoldActive[i] = false;

      rackPositionCommandActive[i] = false;

      rackMode[i] = RACK_REVERSE;
    }

    // ------------------------------------------------------------
    // 0.0: stop and hold current position
    // ------------------------------------------------------------

    else
    {
      relativeMoveActive[i] = false;
      rackPositionCommandActive[i] = false;

      // Preserve the target captured at relative movement
      // completion.

      if (!relativeHoldActive[i])
      {
        targetTicks[i] = current;

        integral[i] = 0;
        previousError[i] = 0;
      }

      rackMode[i] = RACK_HOLD;
    }
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
  const float dt = 0.01f;

  Encoder* encoders[2] = {&enc1, &enc2};

  const int* motors[2] = {RP1, RP2};

  for (int i = 0; i < 2; i++)
  {
    if (rackMode[i] != RACK_HOLD)
      continue;

    long currentTicks = encoders[i]->read();

    long error = targetTicks[i] - currentTicks;

    float P = Kp * error;

    integral[i] += error * dt;

    float I = Ki * integral[i];

    float D =
      Kd * (error - previousError[i]) / dt;

    previousError[i] = error;

    float output = P + I + D;

    if (labs(error) <= 5)
    {
      output = 0;
      integral[i] = 0;
    }

    output = constrain(
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
      analogWrite(motors[i][0], 0);
      analogWrite(motors[i][1], 0);
    }
  }
}

// ================================================================
// TRACTION
// ================================================================

void runTraction()
{
  int left_pwm =
    (int)(traction_left_cmd * 64.0f);

  int right_pwm =
    (int)(traction_right_cmd * 64.0f);

  left_pwm = constrain(left_pwm, -8192, 8192);
  right_pwm = constrain(right_pwm, -8192, 8192);

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

void driveMotor(int pinF, int pinR, int pwm)
{
  if (pwm > 0)
  {
    analogWrite(pinF, 0);
    analogWrite(pinR, pwm);
  }
  else if (pwm < 0)
  {
    analogWrite(pinF, -pwm);
    analogWrite(pinR, 0);
  }
  else
  {
    analogWrite(pinF, 0);
    analogWrite(pinR, 0);
  }
}

// ================================================================
// RACK MOTOR FORWARD
// ================================================================

void motorForward(int rpwm, int lpwm, int pwm)
{
  analogWrite(lpwm, 0);
  analogWrite(rpwm, pwm);
}

// ================================================================
// RACK MOTOR REVERSE
// ================================================================

void motorReverse(int rpwm, int lpwm, int pwm)
{
  analogWrite(lpwm, pwm);
  analogWrite(rpwm, 0);
}

// ================================================================
// STOP RACK MOTORS
// ================================================================

void stopMotor()
{
  analogWrite(RP1[0], 0);
  analogWrite(RP1[1], 0);

  analogWrite(RP2[0], 0);
  analogWrite(RP2[1], 0);
}

// ================================================================
// STOP TRACTION
// ================================================================

void stopTraction()
{
  for (int i = 0; i < 4; i++)
  {
    analogWrite(traction[i][0], 0);
    analogWrite(traction[i][1], 0);
  }
}