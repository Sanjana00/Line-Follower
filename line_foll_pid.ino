#define Kp 0.01 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 15 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used

const int LPWM = 9;
const int LP2 = 7;
const int LP1 = 6;

const int RPWM = 3;
const int RP1 = 5;
const int RP2 = 4;

const int motorPower = 8;

const int irPins[NUM_SENSORS] = {A0, A1, 13, 12, 11, 10, 8, 2};
int irValues = {0, 0, 0, 0, 0, 0, 0, 0};

int lasterror = 0;

void scanIrSensor() {
  for (byte count = 0; count < NUM_SENSORS; count++) {
    irValues[count] = digitalRead(irPins[count]);
  }
}

void setup()
{
  pinMode(LP1, OUTPUT);
  pinMode(LP2, OUTPUT);
  pinMode(RP1, OUTPUT);
  pinMode(RP2, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(RPM, OUTPUT);

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(irPins[i], INPUT);
}

void loop()
{
  scanIrSensor()
  int position = 0 * irValues[0] + 1000 * irValues[1] + 2000 * irValues[2] + 3000 * irValues[3] + 4000 * irValues[4] + 5000 * irValues[5] + 6000 * irValues[6] + 7000 * irValues[7];
  int error = 6500 - position;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;

  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

  {
    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(RP1, HIGH);
    digitalWrite(RP2, LOW);
    analogWrite(RPWM, rightMotorSpeed);
    digitalWrite(motorPower, HIGH);
    digitalWrite(LP1, HIGH);
    digitalWrite(LP2, LOW);
    analogWrite(LPWM, leftMotorSpeed);
  }
}
