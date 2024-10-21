#ifdef ARDUINO
    #include <Arduino.h>
#else
    #include <stdint.h>
    #include <stdbool.h>
#endif

uint8_t getIpsFromDip(uint8_t value, bool isLocal) {
  if (value > 127) return 0;  // error scenario
  if (isLocal) return value + 100;
  else if (value % 2 == 0) return value + 101;  // if even
  else return value + 99;                       // if odd
}
