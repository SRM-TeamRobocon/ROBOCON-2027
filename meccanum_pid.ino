// '''meccanum drive code without rpm pid control '''
// 

#include <micro_ros_arduino.h>
#include <NativeEthernet.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Encoder.h>

// #include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32_multi_array.h>

#define MAX_PWM 250
#define NUM_MOTORS 4

// -------- Drive pin layout (from integrated code) --------
int drive_omni[4][2] = {  //fl - fr - rl -rr
  { 1, 0 },
  { 3, 2 },
  { 5, 4 },
  { 7, 6 }
};

Encoder m[4] = { Encoder(34, 35), Encoder(16, 17), Encoder(41,40), Encoder(26, 27) };

float dt = 0.075;

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
// geometry_msgs__msg__Vector3 msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

IntervalTimer motorTimer;

volatile float motor_pwm[NUM_MOTORS] = {0, 0, 0, 0};

// RPM PID
volatile int pwm_pid[] = { 0, 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0, 0 };
// /1 60 180 0
volatile float kp[] = { 09.0, 9.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.5, 00.50, 00.50 };


// volatile float kp[] = { 0.0, 0.0, 0.0, 0.0 };
// volatile float ki[] = { 0.0, 0.0, 0.0, 0.0 };
// volatile float kd[] = { 0.0, 0.0, 0.0, 0.0 };


float error[] = { 0, 0, 0, 0 };
float eInt[] = { 0, 0, 0, 0 };
float eDer[] = { 0, 0, 0, 0 };
float lastError[] = { 0, 0, 0, 0 };

volatile long oldPosition[4] = { 0, 0, 0, 0 };
volatile long count[4] = { 0, 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[4] = { 0, 0, 0, 0 };

volatile float rpm_rt[4] = { 0, 0, 0, 0 };
float cpr[]={3500.0,3500.0,3500.0, 3500.0};  // CHANGE 

// float vx = 0;
// float vy = 0;
// float omega = 0;

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


// ---------------- Inverse Kinematics ----------------

// void inverseKinematics(float vx, float vy, float omega, float *rpms) {
//   rpms[0] = map(-vx + vy + omega / 2, -255, 255, -MAX_RPM, MAX_RPM);
//   rpms[1] = map(vx + vy + omega / 2, -255, 255, -MAX_RPM, MAX_RPM);
//   rpms[2] = map(vx - vy + omega / 2, -255, 255, -MAX_RPM, MAX_RPM);
//   rpms[3] = map(-vx - vy + omega / 2, -255, 255, -MAX_RPM, MAX_RPM);
// }


// ---------------- Drive Function (same as integrated code) ----------------

void drive(int pwmL, int pwmR, int pwm) {
  analogWrite(pwmL, pwm < 0 ? -pwm : 0);
  analogWrite(pwmR, pwm > 0 ? pwm : 0);
}


// ---------------- Motor Update ----------------

// void motor_update()
// {
//   for (int i = 0; i < NUM_MOTORS; i++)
//   {
//     float command = motor_pwm[i];

//     command = constrain(command, -MAX_PWM, MAX_PWM);

//     drive(
//       drive_omni[i][0],
//       drive_omni[i][1],
//       (int)command
//     );
//   }
// }

void motor_update(){

  for (int i = 0; i < 4; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    rpm_rt[i] = ::count[i] / cpr[i] * 600 * 4.0 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
    // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    // Serial.println();
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }

  float rpm_cmd[4];
//   inverseKinematics(vx, vy, omega, rpm_cmd);
    for(int i=0;i<4;i++){
        rpm_cmd[i] = motor_pwm[i];
    }

    for (int i = 0; i < 4; i++) {
     error[i] = rpm_cmd[i] - rpm_rt[i];
     eDer[i] = (error[i] - lastError[i]) / dt;
     eInt[i] = eInt[i] + error[i] * dt;

     pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);


     //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
//     pwm_pid[i]=pwm_pid[i]%16383;
     pwm_pid[i]=pwm_pid[i]%16383;

     drive(drive_omni[i][0], drive_omni[i][1], (int)pwm_pid[i]);

    //  analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    //  analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);

     lastError[i] = error[i];
   Serial.printf("RPM_output(motor-%d:%0.2f\n", i + 1, rpm_rt[i]);
   Serial.printf("RPM_%d_input:%0.2f\n",i+1, rpm_cmd[i]);
   }
}


// ---------------- ROS Callback ----------------


// void subscription_callback(const void *msgin) {
//   const geometry_msgs__msg__Vector3 *incoming =
//     (const geometry_msgs__msg__Vector3 *)msgin;

//   float x_error = incoming->x;
//   float area_error = incoming->y;

//   // vision → robot velocity
//   vy = constrain(-area_error * 0.01, -255, 255);  // FIX: invert Y direction
//   vx = constrain(x_error, -255, 255);
//   omega = 0;

//   Serial.print("x_error: ");
//   Serial.print(vx);
//   Serial.print("  y_error: ");
//   Serial.println(vy);
// }

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *incoming =
    (const std_msgs__msg__Float32MultiArray *)msgin;

  // Make sure we received at least 4 values
  if (incoming->data.size < NUM_MOTORS)
  {
    return;
  }

  // Directly assign ROS values to motors
  motor_pwm[0] = constrain(
    incoming->data.data[0],
    -MAX_PWM,
    MAX_PWM
  );

  motor_pwm[1] = constrain(
    incoming->data.data[1],
    -MAX_PWM,
    MAX_PWM
  );

  motor_pwm[2] = constrain(
    incoming->data.data[2],
    -MAX_PWM,
    MAX_PWM
  );

  motor_pwm[3] = constrain(
    incoming->data.data[3],
    -MAX_PWM,
    MAX_PWM
  );

  // // Debug output
  // Serial.print("M1: ");
  // Serial.print(motor_pwm[0]);

  // Serial.print("  M2: ");
  // Serial.print(motor_pwm[1]);

  // Serial.print("  M3: ");
  // Serial.print(motor_pwm[2]);

  // Serial.print("  M4: ");
  // Serial.println(motor_pwm[3]);
}

// ---------------- Teensy MAC ----------------

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
  for (uint8_t by = 0; by < 2; by++)
    mac[by] = (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;

  for (uint8_t by = 0; by < 4; by++)
    mac[by + 2] = (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
}
#endif


// ---------------- Setup ----------------

void setup() {
  Serial.begin(115200);
  // Serial.println("ok");
  delay(2000);
  analogWriteResolution(14);

  byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };

#if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
#endif

  IPAddress arduino_ip(192, 168, 1, 150);
  IPAddress agent_ip(192, 168, 1, 100);

   for (int i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(drive_omni[i][0], OUTPUT);
    pinMode(drive_omni[i][1], OUTPUT);

    analogWrite(drive_omni[i][0], 0);
    analogWrite(drive_omni[i][1], 0);
  }

  set_microros_native_ethernet_udp_transports(
    arduino_mac,
    arduino_ip,
    agent_ip,
    9999);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "teensy_align",
    "",
    &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/target_rpm"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  std_msgs__msg__Float32MultiArray__init(&msg);

  msg.data.data = (float *)malloc(NUM_MOTORS * sizeof(float));
  msg.data.size = NUM_MOTORS;
  msg.data.capacity = NUM_MOTORS;

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));

  motorTimer.begin(motor_update, 75000);
  // Serial.print("hi");
}

void readPIDTuning()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    float newKp, newKi, newKd;
    int motorno;

    if (sscanf(command.c_str(), "%d %f %f %f", &motorno,
               &newKp, &newKi, &newKd) == 4)
    {
        kp[motorno] = newKp;
        ki[motorno] = newKi;
        kd[motorno] = newKd;

      // Reset integral and derivative history
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        eInt[i] = 0;
        lastError[i] = 0;
      }

      // Serial.println("PID updated:");

      // Serial.print("Motor = ");
      // Serial.println(motorno);

      // Serial.print("Kp = ");
      // Serial.println(newKp);

      // Serial.print("Ki = ");
      // Serial.println(newKi);

      // Serial.print("Kd = ");
      // Serial.println(newKd);
    }
    else
    {
      // Serial.println("Invalid command.");
      // Serial.println("Use: pid Kp Ki Kd");
      // Serial.println("Example: pid 2.5 10 0.2");
    }
  }
}


// ---------------- Loop ----------------

void loop() {
  readPIDTuning();
  RCSOFTCHECK(
    rclc_executor_spin_some(
      &executor,
      RCL_MS_TO_NS(10)));
}

//ros2 run micro_ros_agent micro_ros_agent udp4 --port 9999