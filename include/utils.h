#include <Arduino.h>

byte CRC8(const void *data, byte len);
void printStruct(const void * data, byte len);
unsigned long millisSince(unsigned long time);
unsigned long secondsSince(unsigned long time);
