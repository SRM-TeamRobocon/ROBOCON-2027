#include <Servo.h>

Servo servos[4];

void setup() {
  Serial.begin(9600);

  servos[0].attach(8);
  // servos[1].attach(8);
  // servos[2].attach(11);
  // servos[3].attach(29);

  servos[0].write(0);
  //servos[1].write(50);
  // servos[2].write(90);
  // servos[3].write(30);

  // servos[0].attach(8);
  // servos[0].write(40);

  Serial.println("Format: servo angle");
  Serial.println("Example: 1 150");
}

void loop() {
  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    int spacePos = input.indexOf(' ');

    if (spacePos == -1) {
      Serial.println("Invalid format");
      return;
    }

    String servoStr = input.substring(0, spacePos);
    String angleStr = input.substring(spacePos + 1);

    int servoNum = servoStr.toInt();
    int angle = angleStr.toInt();

    if (servoNum < 1 || servoNum > 4) {
      Serial.println("Servo number must be 1-4");
      return;
    }

    angle = constrain(angle, 0, 180);

    servos[servoNum - 1].write(angle);

    Serial.print("Servo ");
    Serial.print(servoNum);
    Serial.print(" -> ");
    Serial.println(angle);
  }
}