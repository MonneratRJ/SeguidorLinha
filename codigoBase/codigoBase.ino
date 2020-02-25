#include <QTRSensors.h>

#define Kp 0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. (Note: Kp < Kd)

// MaxSpeeds should be tuned if the robot seems to turn faster with one motor than with the other...
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot

// BaseSpeeds should be tuned if the robot seems to be "turning" while it sould be going straight... 
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line

QTRSensors qtr;

const uint8_t L1 = 5, L2 = 6, R1 = 9, R2 = 10;
const uint8_t SensorCount = 6;
const uint8_t EmitterPin = A6;
uint16_t sensorValues[SensorCount];
uint8_t position, error, lastError, motorSpeed;

void setup()
{
  // Setting A6 as an OUTPUT for the LEDON switch wich will be HIGH the entire run.
  pinMode(A6, OUTPUT);
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(EmitterPin);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  delay(500);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  // uint16_t position = qtr.readLineBlack(sensorValues);
  position = qtr.readLineWhite(sensorValues);
  error = position - 2500;
  
  motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
  // Prevents the motors from going beyond MAX speed
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;

  // Keeps the motor speeds positive
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;


  // Sends the power to the wheels
  digitalWrite(R1, LOW);
  analogWrite(R2, rightMotorSpeed);
  digitalWrite(L1, LOW);
  analogWrite(L2, leftMotorSpeed);
}
