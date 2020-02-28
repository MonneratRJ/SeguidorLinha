const uint8_t L1 = 6, L2 = 5, R1 = 10, R2 = 9;

void setup() {
  // put your setup code here, to run once:
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  
  analogWrite(R1, 150);
  analogWrite(L1, 150);
  digitalWrite(R2, LOW);
  digitalWrite(L2, LOW);
  delay(3000);
}

void loop() {

}
