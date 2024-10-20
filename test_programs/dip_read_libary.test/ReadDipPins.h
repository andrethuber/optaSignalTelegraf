/*
  ReadDipPins.h - Library to init and read multiple pins like a DIP switch, and return a integer value based on the decimal representation of the input pins read as binary.
  Created by andrethuber, 2024-10-19.
  Released into the public domain.
*/

#ifndef ReadDipPins_h
#define ReadDipPins_h

#include "Arduino.h"

class ReadDipPins {
public:
  ReadDipPins();
  void begin(uint8_t pins[]);
  uint8_t get(uint8_t pins[]);
};

#endif
