void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  //Serial.begin(9600); //Enable Serial Communications
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(10, 0);
  analogWrite(5, 255);
  delay(500);

  analogWrite(5, 0);
  analogWrite(9, 255);
  delay(500);

  analogWrite(9, 0);
  analogWrite(6, 255);
  delay(500);

  analogWrite(6, 0);
  analogWrite(10, 255);
  delay(500);
}
