// int trigpin = 20;   //1
// int echopin = 21;

// int trigpin = 30;   //2
// int echopin = 31;

// int trigpin = 23;   //3
// int echopin = 22;

// int trigpin = 40;    //4
// int echopin = 41;

int trigpin = 28;       //5   //fluctuating 
int echopin = 29;

// int trigpin = 13;     //6
// int echopin = 12;

// int trigpin = 14;    //7
// int echopin = 15;

// int trigpin = 33;    //8
// int echopin = 32;

// int trigpin = 27;    //9
// int echopin = 26;

// int trigpin = 37;   //10
// int echopin = 36;

void pinsetup(int trig, int echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

float distance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  int duration = pulseIn(echo, HIGH);
  float dist = (duration / 2) / 29.1;  // cm
  return dist;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("starting  ....");
  pinsetup(trigpin, echopin);
}

void loop() {
  // put your main code here, to run repeatedly:
  float dist = distance(trigpin, echopin);
  Serial.print("distance :");
  Serial.println(dist);
  delay(100);
}
