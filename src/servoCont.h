int minPos = 5;  //according to garrett engineer, production applications are limited from 5-85% range (0-5v) hall sensor
int maxPos = 85; //possible hardware damage if driven past 90%
int driveSpeed = 250;




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
void forward(speed) {
  digitalWrite(MOTOR_IN1, LOW);
  analogWrite(MOTOR_IN2, speed);
}

void reverse(speed) {
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_IN1, speed);
}

void brake() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH);
}





///////////////////////////

double kp = 5 , ki = 1 , kd = 0.01;             // modify for optimal performance
double feedback = 0, output = 0, setpoint = 0;
long temp;
volatile long encoderPos = 0;
PID myPID(&feedback, &output, &setpoint, kp, ki, kd, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'

void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  servoPID.SetMode(AUTOMATIC);
  servoPID.SetSampleTime(1);
  servoPID.SetOutputLimits(-255, 255);
  Serial.begin (115200);                              // for debugging
}

void loop() {
  setpoint = analogRead(0) * 5;                       // modify to fit motor and encoder characteristics, potmeter connected to A0
  input = encoderPos ;                                // data from encoder
  // Serial.println(encoderPos);                      // monitor motor position
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 0);
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));                        // drive motor CCW

////////////////////////////





void loop() {

  setpoint = analogRead(A2);  //use for testing using potentiometer...
  feedback = encoderPos ;                                // data from encoder
  // Serial.println(encoderPos);                      // monitor motor position
  servoPID.Compute();                                    // calculate new output
  servoDrive(output);                                     // drive L298N H-Bridge module
}

void servoDrive(int out) {                                // to H-Bridge board
  if (out > 0) {
    forward(out);
  }
  else {
    reverse(abs(out));
}
