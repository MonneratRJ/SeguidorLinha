#include <QTRSensors.h>

#define Kp 0.3 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. (Note: Kp < Kd)

// MaxSpeeds should be tuned if the robot seems to turn faster with one motor than with the other...
#define rightMaxSpeed 255 // max speed of the robot
#define leftMaxSpeed 255 // max speed of the robot

// BaseSpeeds should be tuned if the robot seems to be "turning" while it sould be going straight... 
#define rightBaseSpeed 200 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 200  // this is the speed at which the motors should spin when the robot is perfectly on the line

QTRSensors qtr;

const uint8_t L1 = 6, L2 = 5, R1 = 10, R2 = 9;
const uint8_t SensorCount = 6;
const uint8_t EmitterPin = A6;
uint16_t sensorValues[SensorCount];
int position, error, lastError, motorSpeed;

void setup()
{
  // Setting A6 as an OUTPUT for the LEDON switch wich will be HIGH the entire run.
  //pinMode(A6, OUTPUT);
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  //qtr.setEmitterPin(EmitterPin);

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

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
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

  // Debug
  // Serial.print("Posicao: ");
  // Serial.println(position);
  // Serial.print("ERRO: ");
  // Serial.println(error);
  // Serial.print("Esquerda: ");
  // Serial.println(leftMotorSpeed);
  // Serial.print("Direita: ");
  // Serial.println(rightMotorSpeed);
  // Serial.println();Serial.println();
  // delay(250);

  // Sends the power to the wheels
  analogWrite(R1, rightMotorSpeed);
  digitalWrite(R2, LOW);
  analogWrite(L1, leftMotorSpeed);
  digitalWrite(L2, LOW);
  Serial.println(position);
}
