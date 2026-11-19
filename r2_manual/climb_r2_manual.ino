#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <CrashReport.h>
#include <NativeEthernet.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>

#include <Encoder.h>
#include <IntervalTimer.h>

// ================= NETWORK CONFIG =================

IPAddress teensy_ip(192, 168, 1, 151);
IPAddress agent_ip(192, 168, 1, 100);

const uint16_t AGENT_PORT = 9999;


// ================= CLIMB PRESET ANGLES =================

// Boolean TRUE
const int SHOULDER_ANGLE_TRUE = -360;
const int ELBOW_ANGLE_TRUE = -360;

// Boolean FALSE
const int SHOULDER_ANGLE_FALSE = 0;
const int ELBOW_ANGLE_FALSE = 0;


// ================= ARM PARAMETERS =================

float ticks_per_deg = 32000.0 / 360.0;

const float ARM_DT = 0.075f;
const float MAX_ARM_PWM = 10000.0f;


// ================= SHOULDER =================

Encoder shoulderEnc(19,18);

int shoulderRPWM = 3;
int shoulderLPWM = 4;

float shoulderKP = 3.0;
float shoulderKI = 0.0;
float shoulderKD = 0.0;

volatile float shoulderErr = 0;
volatile float shoulderPrevErr = 0;
volatile float shoulderInteg = 0;

volatile long shoulderSP_ticks = 0;

IntervalTimer shoulderTimer;


// ================= ELBOW =================

Encoder armEnc(21, 20);

int armRPWM = 37;
int armLPWM = 36;

float armKP = 3.0;
float armKI = 0.0;
float armKD = 0.0;

volatile float armErr = 0;
volatile float armPrevErr = 0;
volatile float armInteg = 0;

volatile long armSP_ticks = 0;

IntervalTimer armTimer;


// ================= MICRO-ROS OBJECTS =================

rcl_subscription_t sub_climb;

std_msgs__msg__Bool msg_climb;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


// ================= ERROR HANDLING =================

#define RCCHECK(fn) \
{ \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
        while (1); \
    } \
}

#define RCSOFTCHECK(fn) \
{ \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) {} \
}


// ================= MOTOR DRIVER =================

void runBTS(int RPWM, int LPWM, float val)
{
    int pwm = abs((int)val);

    pwm = constrain(pwm, 0, (int)MAX_ARM_PWM);

    if (val > 0)
    {
        analogWrite(RPWM, pwm);
        analogWrite(LPWM, 0);
    }
    else if (val < 0)
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, pwm);
    }
    else
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
    }
}


// ================= SHOULDER PID =================

void shoulderPIDLoop()
{
    long pos = shoulderEnc.read();

    float error = (float)(shoulderSP_ticks - pos);

    float integral = shoulderInteg + (error * ARM_DT);

    float derivative =
        (error - shoulderPrevErr) / ARM_DT;

    float output =
        (shoulderKP * error) +
        (shoulderKI * integral) +
        (shoulderKD * derivative);

    output = constrain(
        output,
        -MAX_ARM_PWM,
        MAX_ARM_PWM
    );

    shoulderErr = error;
    shoulderInteg = integral;
    shoulderPrevErr = error;

    runBTS(
        shoulderRPWM,
        shoulderLPWM,
        output
    );
}


// ================= ELBOW PID =================

void armPIDLoop()
{
    long pos = armEnc.read();

    float error = (float)(armSP_ticks - pos);

    float integral = armInteg + (error * ARM_DT);

    float derivative =
        (error - armPrevErr) / ARM_DT;

    float output =
        (armKP * error) +
        (armKI * integral) +
        (armKD * derivative);

    output = constrain(
        output,
        -MAX_ARM_PWM,
        MAX_ARM_PWM
    );

    armErr = error;
    armInteg = integral;
    armPrevErr = error;

    runBTS(
        armRPWM,
        armLPWM,
        output
    );
}


// ================= CLIMB CALLBACK =================
void climbCallback(const void *msgin)
{
    const std_msgs__msg__Bool *msg =
        (const std_msgs__msg__Bool *)msgin;

    int shoulderAngle;
    int elbowAngle;

    if (msg->data == true)
    {
        // TRUE: Shoulder = 90, Elbow = -90

        shoulderAngle = SHOULDER_ANGLE_TRUE;
        elbowAngle = ELBOW_ANGLE_TRUE;

        Serial.println("CLIMB POSITION 1");
    }
    else
    {   
        // FALSE: Shoulder = 0, Elbow = 10

        shoulderAngle = SHOULDER_ANGLE_FALSE;
        elbowAngle = ELBOW_ANGLE_FALSE;

        Serial.println("CLIMB POSITION 2");
    }

    // Convert angles to encoder ticks

    long newShoulderSP =
        (long)(shoulderAngle * ticks_per_deg);

    long newElbowSP =
        (long)(elbowAngle * ticks_per_deg);

    // Update setpoints

    noInterrupts();

    shoulderSP_ticks = newShoulderSP;
    armSP_ticks = newElbowSP;

    // Reset PID states

    shoulderInteg = 0;
    shoulderPrevErr = 0;

    armInteg = 0;
    armPrevErr = 0;

    interrupts();

    Serial.print("Shoulder target: ");
    Serial.println(shoulderAngle);

    Serial.print("Elbow target: ");
    Serial.println(elbowAngle);
}


// ================= TEENSY MAC =================

#if defined(ARDUINO_TEENSY41)

void get_teensy_mac(uint8_t *mac)
{
    for (uint8_t by = 0; by < 2; by++)
    {
        mac[by] =
            (HW_OCOTP_MAC1 >> ((1 - by) * 8)) & 0xFF;
    }

    for (uint8_t by = 0; by < 4; by++)
    {
        mac[by + 2] =
            (HW_OCOTP_MAC0 >> ((3 - by) * 8)) & 0xFF;
    }
}

#endif


// ================= SETUP =================

void setup()
{
    Serial.begin(115200);

    if (CrashReport)
    {
        Serial.print(CrashReport);
    }

    // PWM configuration

    analogWriteResolution(14);

    analogWriteFrequency(armRPWM, 9000);
    analogWriteFrequency(armLPWM, 9000);

    analogWriteFrequency(shoulderRPWM, 9000);
    analogWriteFrequency(shoulderLPWM, 9000);


    // Motor pins

    pinMode(armRPWM, OUTPUT);
    pinMode(armLPWM, OUTPUT);

    pinMode(shoulderRPWM, OUTPUT);
    pinMode(shoulderLPWM, OUTPUT);

    analogWrite(armRPWM, 0);
    analogWrite(armLPWM, 0);

    analogWrite(shoulderRPWM, 0);
    analogWrite(shoulderLPWM, 0);


    // Start PID timers

    armTimer.begin(armPIDLoop, 75000);

    shoulderTimer.begin(
        shoulderPIDLoop,
        75000
    );


    // ================= MICRO-ROS SETUP =================

    byte arduino_mac[] =
    {
        0xAA, 0xBB, 0xCC,
        0xEE, 0xDD, 0xFF
    };

    #if defined(ARDUINO_TEENSY41)

    get_teensy_mac(arduino_mac);

    #endif


    // Ethernet transport

    set_microros_native_ethernet_udp_transports(
        arduino_mac,
        teensy_ip,
        agent_ip,
        AGENT_PORT
    );


    // Wait for micro-ROS agent

    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
    {
        Serial.println("Waiting for micro-ROS agent...");

        delay(500);
    }

    Serial.println("Connected to micro-ROS agent!");


    // Initialize micro-ROS

    allocator = rcl_get_default_allocator();

    RCCHECK(
        rclc_support_init(
            &support,
            0,
            NULL,
            &allocator
        )
    );


    // Initialize node

    RCCHECK(
        rclc_node_init_default(
            &node,
            "teensy_climb_controller",
            "",
            &support
        )
    );


    // ================= CLIMB SUBSCRIPTION =================

    RCCHECK(
        rclc_subscription_init_default(
            &sub_climb,
            &node,

            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Bool
            ),

            "climb_status"
        )
    );


    // Initialize Boolean message

    std_msgs__msg__Bool__init(&msg_climb);


    // ================= EXECUTOR =================

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );


    // Register subscription callback

    RCCHECK(
        rclc_executor_add_subscription(
            &executor,
            &sub_climb,
            &msg_climb,
            &climbCallback,
            ON_NEW_DATA
        )
    );


    Serial.println("--------------------------------");
    Serial.println("TEENSY CLIMB CONTROLLER STARTED");
    Serial.println("Subscribed to: /climb");
    Serial.println("Message type: std_msgs/Bool");
    Serial.println("--------------------------------");

    // Serial.print("Position 1: ");
    // Serial.println(CLIMB_ANGLE_1);

    // Serial.print("Position 2: ");
    // Serial.println(CLIMB_ANGLE_2);
}


// ================= LOOP =================

void loop()
{
    RCSOFTCHECK(
        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(1)
        )
    );
}