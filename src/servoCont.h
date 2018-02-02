int minPos = 5;  //according to garrett engineer, production applications are limited from 5-85% range (0-5v) hall sensor
int maxPos = 85; //possible hardware damage if driven past 90%
int driveSpeed = 250;
int hysteresis = 12;

//using Adafruit DRV8871 Breakout

#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define feedback 11

void setup() {
  Serial.begin(9600);

  Serial.println("DRV8871 test");

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(feedback, INPUT);
}

//  motor direction definition
void forward() {
  digitalWrite(MOTOR_IN1, LOW);
  analogWrite(MOTOR_IN2, driveSpeed);
}

void reverse() {
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_IN1, driveSpeed);
}

void brake() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}


void loop() {

  targetPos = analogRead(A2);  //use for testing using potentiometer...
  if (targetPos > feedback-hysteresis && targetPos< feedback+hysteresis) {
    brake();
  } else
  if (feedback < targetPos) {
    forward();
  } else if (feedback > targetPos) {
    reverse();
  }
}
