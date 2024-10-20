bool blink = false;

void setup() {
  Serial.begin(9600);
  pinMode(LED_D0, OUTPUT);
  Serial.println("'void setup' complete");
}

void loop() {
  digitalWrite(LED_D0, blink);
  Serial.println(blink);
  blink = !blink;
  delay(500);
}
