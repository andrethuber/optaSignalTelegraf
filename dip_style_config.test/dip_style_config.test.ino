
#include "ReadDipPins.h"

uint8_t dipPins[] = { A2, A3, A4, A5, A6, A7 };

ReadDipPins readDipPins;

void setup() {
  Serial.begin(9600);
  readDipPins.begin(dipPins);
}

void loop() {
  uint8_t value = readDipPins.get(dipPins);
  Serial.println(value);
  delay(1000);
}

/*
void initDipPins(uint8_t pins[]) {
  for ( uint8_t i = 0; i < sizeof(pins); i++) {
    pinMode(pins[i], INPUT);
  }
}

uint8_t readDipPins(uint8_t pins[]) {
  uint8_t count = sizeof(dipPins);
  bool dipState[count];
  uint8_t dipValue = 0;
  for ( uint8_t i = 0; i < count; i++) {
    dipState[i] = digitalRead(pins[i]);
    dipValue = dipValue + pow(2, count - 1 - i) * dipState[i];
  }
  return dipValue;
}
*/