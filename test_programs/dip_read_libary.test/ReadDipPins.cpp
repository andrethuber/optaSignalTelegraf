/*
  ReadDipPins.h - Library to init and read multiple pins like a DIP switch, and return a integer value based on the decimal representation of the input pins read as binary.
  Created by andrethuber, 2024-10-19.
  Released into the public domain.
*/

#include "Arduino.h"
#include "ReadDipPins.h"

ReadDipPins::ReadDipPins(uint8_t pins[]) {
}

void ReadDipPins::begin() {
  for ( uint8_t i = 0; i < sizeof(pins); i++) {
    pinMode(pins[i], INPUT);
  }
}

uint8_t ReadDipPins::get() {
  uint8_t dipCount = sizeof(pins);
  bool dipState[dipCount];
  uint8_t dipValue = 0;
  for ( uint8_t i = 0; i < dipCount; i++) {
    dipState[i] = digitalRead(pins[i]);
    dipValue = dipValue + pow(2, dipCount - 1 - i) * dipState[i];
  }
  return dipValue;
}
