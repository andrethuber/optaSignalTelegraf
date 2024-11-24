uint16_t analogValue;

void setup() {
  // Phone ring in/out:
  pinMode(A2, INPUT);
  Serial.begin(300);
}

void loop() {
  analogValue = analogRead(A2);
  //Serial.print("0, 1024, ");
  Serial.println(analogValue);

}
