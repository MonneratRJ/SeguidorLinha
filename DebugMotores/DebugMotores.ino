const uint8_t L1 = 9, L2 = 10, R1 = 13, R2 = 14;

void setup() {
  // put your setup code here, to run once:
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  
  //Serial.begin(9600); //Enable Serial Communications
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(R2, 0);
  analogWrite(L1, 255);
  delay(500);

  analogWrite(L1, 0);
  analogWrite(R1, 255);
  delay(500);

  analogWrite(R1, 0);
  analogWrite(L2, 255);
  delay(500);

  analogWrite(L2, 0);
  analogWrite(R2, 255);
  delay(500);
}
