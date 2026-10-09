const int SOLENOID_PIN = 18;

void setup() {
  Serial.begin(115200);

  pinMode(SOLENOID_PIN, OUTPUT);

  // Start with the solenoid switched OFF
  digitalWrite(SOLENOID_PIN, LOW);

  Serial.println("Solenoid Gripper Ready");
  Serial.println(" 'o' to open");
  Serial.println(" 'c' to close");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'o') {
      digitalWrite(SOLENOID_PIN, HIGH);
      Serial.println("Gripper ACTIVATED");
    }

    else if (command == 'c') {
      digitalWrite(SOLENOID_PIN, LOW);
      Serial.println("Gripper RELEASED");
    }
  }
}
