#include <QTRSensors.h>

#define Kp .19 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 1.12 // experiment to determine this, slowly increase the speeds and adjust this value. (Note: Kp < Kd)
#define lapSeconds 18 // this will determine how many seconds the bot will be running, remember this is in SECONDS, not millis...

// MaxSpeeds should be tuned if the robot seems to turn faster with one motor than with the other...
#define rightMaxSpeed 250 
#define leftMaxSpeed 250

// BaseSpeeds should be tuned if the robot seems to be "turning" while it sould be going straight...
#define rightBaseSpeed 142 
#define leftBaseSpeed 150

QTRSensors qtr;

const uint8_t L1 = 6, L2 = 5, R1 = 10, R2 = 9;
const uint8_t SensorCount = 6;
const uint8_t EmitterPin = A6;
uint16_t sensorValues[SensorCount];
uint8_t rightMotorBrakes = 0, leftMotorBrakes = 0;
int position, error, lastError, motorSpeed;

// Variable to keep LAP TIME (no start/stop sensor), this will be used in milliseconds and started by "startLap()" function.
long lapTime;

long startLap(int time) {
  lapTime = millis() + time * 1000;
  return lapTime;
}

void setup()
{
  // Setting A6 as an OUTPUT for the LEDON switch wich will be HIGH the entire run. (OPTIONAL)
  //pinMode(A6, OUTPUT);
  
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  // If you need to use a pin to turn ON/OFF the sensors, set it on pinMode then uncomment this line below:
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
  // Serial.begin(9600);
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(qtr.calibrationOn.minimum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();

  // // print the calibration maximum values measured when emitters were on
  // for (uint8_t i = 0; i < SensorCount; i++)
  // {
  //   Serial.print(qtr.calibrationOn.maximum[i]);
  //   Serial.print(' ');
  // }
  // Serial.println();
  // Serial.println();

  lapTime = startLap(lapSeconds);
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

  // If the speeds are negative, turn into braking proportional to the negative speed received
  if (rightMotorSpeed < 0) rightMotorBrakes = rightMotorSpeed * -1;
  if (leftMotorSpeed < 0) leftMotorBrakes = leftMotorSpeed * -1;

  // Debug Avançado...
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

  if(millis() < lapTime) {
    // Verifies if a sharp turn needs braking
    if(rightMotorSpeed < 0 || leftMotorSpeed < 0) {
      // Verifies which side will brake
      if(rightMotorSpeed < leftMotorSpeed){
        // Apply breaking on RIGHT MOTOR.
        analogWrite(R1, rightMotorBrakes);
        digitalWrite(R2, rightMotorBrakes);
        analogWrite(L1, leftMotorSpeed);
        digitalWrite(L2, LOW);
      } else {
        // Apply breaking on LEFT MOTOR.
        analogWrite(R1, rightMotorSpeed);
        digitalWrite(R2, LOW);
        analogWrite(L1, leftMotorBrakes);
        digitalWrite(L2, leftMotorBrakes);
      }
    } else {
      // Sends the power to the wheels
      analogWrite(R1, rightMotorSpeed);
      digitalWrite(R2, LOW);
      analogWrite(L1, leftMotorSpeed);
      digitalWrite(L2, LOW);
    }
  } else {
    // Lap time is OVER, STOP.
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(R1, LOW);
    digitalWrite(R2, LOW);
    digitalWrite(L1, LOW);
    digitalWrite(L2, LOW);
  }
}
