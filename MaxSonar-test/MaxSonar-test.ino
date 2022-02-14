void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(A0) / 2.0);
  delay(250);
}
