#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(30, 31);
// LIFTING ACTUATOR
#define LPWM 2         // PWM pin forward         //change pinouts as required
#define RPWM 3           // PWM pin reverse
const int MAX_RPM = 50;  // Motor’s rated RPM


// KFS Y-AXIS
/*#define PWM 0    // PWM pin forward
#define DIR 1    // PWM pin reverse
const int MAX_RPM = 350;  // Motor’s rated RPM
*/



int targetRPM = 0;  // Current target
int pwmValue = 0;   // Mapped PWM

void setup() {
  Serial.begin(9600);
  //pinMode(13, OUTPUT);
  //digitalWrite(13, HIGH);

  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);

  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);

  Serial.println("Actuator Control Ready");
  Serial.println("Send RPM value (e.g., 400 for forward, -400 for reverse, 0 to stop)");
}

void loop() {

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      int newRPM = input.toInt();  // convert to integer
      targetRPM = constrain(newRPM, -MAX_RPM, MAX_RPM);

      Serial.print("New target RPM set to: ");
      Serial.println(targetRPM);
    }
  }


  if (targetRPM > 0) {
    pwmValue = map(targetRPM, 0, MAX_RPM, 0, 255);
    runMotorForward(pwmValue);
    long newPosition = myEnc.read();
    Serial.println(newPosition);
    
  } else if (targetRPM < 0) {
    pwmValue = map(abs(targetRPM), 0, MAX_RPM, 0, 255);
    runMotorReverse(pwmValue);
    //long newPosition = myEnc.read();
    //Serial.println(newPosition);
    
  } else {
    stopMotor();
  }
}
// LIFTING ACTUATOR
// #define LPWM 0    // PWM pin forward         //change pinouts as required
// #define RPWM 1    // PWM pin reverse
// const int MAX_RPM = 350;  // Motor’s rated RPM


// KFS Y-AXIS
/*#define LPWM 0    // LPWM pin forward
#define DIR 1    // LPWM pin reverse
…  digitalWrite(DIR, HIGH);
}

void stopMotor() {
  analogWrite(PWM, 0);
  digitalWrite(DIR, HIGH);
}*/



void runMotorForward(int pwm) {
  analogWrite(RPWM, pwm);
  analogWrite(LPWM, 0);
}

void runMotorReverse(int pwm) {
  analogWrite(LPWM, pwm);
  analogWrite(RPWM, 0);
}

void stopMotor() {
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 0);
}
