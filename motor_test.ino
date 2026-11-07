#include "USBHost_t36.h"
#include <Encoder.h>
IntervalTimer rpm_timer;
IntervalTimer ps4_timer;
const int ledPin = LED_BUILTIN;  // the pin with a LED

// R1 DRIVE
// Encoder myEnc(27,26); // bl
// int m4_rpwm = 0;
// int m4_lpwm = 1;

// Encoder myEnc(29,10); // br
// int m4_rpwm = 2;
// int m4_lpwm = 3;

// Encoder myEnc(15,27); // fr
// int m4_rpwm = 5;
// int m4_lpwm = 4;

Encoder myEnc(29, 28);  //fl
int m4_rpwm = 4;  
int m4_lpwm = 5; 

// Encoder myEnc(29,28); //deadwheel fwd
// int m4_rpwm = 36;
// int m4_lpwm = 37; 

// //Encoder myEnc(100,100); //strafe
// int m4_rpwm = 7;
// int m4_lpwm = 6; 




volatile float rpm=0;
int val = 0;

int res=pow(2,14)-1;
int duty_cycle=25;//in percentage
int max_pwm =(int)(duty_cycle/100.0*res); //6v--250rpm

//int res=pow(2,14)-1;
//int duty_cycle=25;//in percentage
//int max_pwm =(int)(duty_cycle/100.0*res); //6v--250rpm
void drive(int pwm_val, int rpwm, int lpwm)
{
  analogWrite(rpwm, pwm_val <= 0 ? 0 : pwm_val);
  analogWrite(lpwm, pwm_val < 0 ? abs(pwm_val) : 0);
 // Serial.printf("Dir: %d\n", (pwm_val <= 0 ? LOW : HIGH));
  //Serial.printf("PWM: %d \n", abs(pwm_val),);
}

// USBHost myusb;
// JoystickController joystick1(myusb);
// // BluetoothController bluet(myusb, true, "0000");
// BluetoothController bluet(myusb);   // version assumes it already was paireduint32_t buttons_prev = 0;

uint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;



void setup() {

  Serial.begin(9600);
  // while (!Serial) ; // wait for Arduino Serial Monitor

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  // myusb.begin();

 // analogWriteFrequency(m2_pwm, 9000);
  
  // pinMode(13, OUTPUT);
  // digitalWrite(13,HIGH);
  pinMode(m4_lpwm,OUTPUT);
  pinMode(m4_rpwm,OUTPUT);
  
  Serial.begin(9600);
  ps4_timer.priority(0);
  rpm_timer.priority(1);
  ps4_timer.begin(ps4_input,75000);
  rpm_timer.begin(calc_rpm, 75000);// calcRpm() to run every 0.1 seconds....us
  analogWriteResolution(14);

  analogWriteFrequency(m4_lpwm, 9000);
  
//  analogWrite(m4_pwm,6000);
//  Serial.println("move");
//  analogWrite(m4_dir,res);
  
}
 long oldPosition  = 0;
int ledState = LOW;
volatile unsigned long count = -999; // use volatile for shared variables
volatile long newPosition;


void calc_rpm() {
  newPosition=myEnc.read();
  count=abs(newPosition-oldPosition);
  rpm=count/3500.0*600*4/3;
  rpm*=newPosition<oldPosition?-1:1;
  Serial.printf("RPM: %f   ",rpm);
  Serial.println(newPosition);
  count=0;
  oldPosition=newPosition;
Serial.println(myEnc.read());
//
//Serial.println(millis());
}


void ps4_input(){
  // String input = Serial.readStringUntil('\n');

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    val=input.toInt();
                 
  }
// myusb.Task();
//   if (joystick1.available()) {
//     for (uint8_t i = 0; i < 64; i++) {
//       psAxis_prev[i] = psAxis[i];
//       psAxis[i] = joystick1.getAxis(i);
//     }

//     buttons = joystick1.getButtons();

//     int psAxisX=0;
// int psAxisY=0;
// int w= 0;
// if(psAxis[0]<125)
//          psAxisX=map(psAxis[0],125,0,0,255);
      
//       else if(psAxis[0]>135)
//         psAxisX=map(psAxis[0],135,255,0,-255);
//       else
//          psAxisX=0;
      
//       if(psAxis[1]>135)
//         psAxisY=map(psAxis[1],135,255,0,-255);
      
//       else if(psAxis[1]<125)
//          psAxisY=map(psAxis[1],125,0,0,255);
//       else
//          psAxisY=0;
//       if(psAxis[2]>135)
//         w=map(psAxis[2],135,255,0,-255);
      
//       else if(psAxis[2]<125)
//          w=map(psAxis[2],125,0,0,255);
//       else
//          w=0;
//     int y = psAxisY;
//     int x= psAxisX;
          Serial.println(val);
//             //int y = map(left_y, -128, 128,max_pwm, -max_pwm);
          Serial.printf(" RPWM:  %d\n", val*255);
          Serial.printf(" LPWM:  %d\n", -(val*255));
          int pwm_cmd = 64*val;
          pwm_cmd = constrain(pwm_cmd, -8500, 8500);
          drive(pwm_cmd, m4_rpwm, m4_lpwm);
}

          
float rpmCopy=0;
// The main program will print the blink count
// to the Arduino Serial Monitor
void loop() {

    
          

//ps4_input();
  

 
}