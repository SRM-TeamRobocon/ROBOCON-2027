#include <VescUart.h>
#include <AS5600.h>

// Pass the address of Wire2 to the library
//AS5600 as5600(&Wire1);

VescUart UART;
const long ENC_RES = 4096;

float rpm = 0;
void setup() {
  // Serial is the USB cable to your PC (for debugging)
  Serial.begin(9600);
  Serial.println("starting ....");

  // Serial1 is the hardware pins (RX1/TX1)
  // Teensy handles 115200 baud perfectly
  Serial1.begin(115200);
  while(!Serial){
    Serial.println("not connected");
  }

  UART.setSerialPort(&Serial1);
  //Wire1.begin();

  // Optional: check connection
  //as5600.begin();
  /*if (!as5600.isConnected()) {
    Serial.println("AS5600 not connected ");
    while (1)
      ;
  }*/
}

void loop() {


  if (Serial.available() > 0) {
    rpm = Serial.parseFloat();  // Read the number entered

    // Clear the buffer of any leftover characters (like \n or \r)
    while (Serial.available() > 0) Serial.read();

    Serial.print("Moving at rpm: ");
    Serial.println(rpm);
  }
  float targetERPM = rpm * 7;
  UART.setRPM(targetERPM);

  // Optional: Print VESC data to your PC Serial Monitor
  if (UART.getVescValues()) {
    Serial.print("Voltage: ");
    Serial.println(UART.data.inpVoltage);  // Changed from v_in to inpVoltage

    Serial.print("Current: ");
    Serial.println(UART.data.avgInputCurrent);

    Serial.print("RPM: ");
    Serial.println(UART.data.rpm);
  }
  // long pos = as5600.readAngle();
  // float currentDeg = pos * (360.0 / ENC_RES);

  //Serial.println(currentDeg);
  //delay(100);

  delay(20);
}