#ifndef _CONST_H
#define _CONST_H

#include <Arduino.h>

// #define DEBUG // Uncomment DEBUG if you need to debug the remote

#ifdef DEBUG
  #define debug(x) Serial.println (x)
#else
  #define debug(x)
#endif

/*
  Connect receiver and open Serial Monitor (Cmd+Shift+M),
  it will display the correct chip ID.
*/
const uint32_t boardAddress = 0xc89cb368;

// default remote configuration
#ifdef ESP32
  const int MIN_HALL = 400;
  const int CENTER_HALL = 740;
  const int MAX_HALL = 1023;
#endif
#ifdef ARDUINO_SAMD_ZERO // Feather M0
  const int MIN_HALL = 18;
  const int CENTER_HALL = 325;
  const int MAX_HALL = 629;
#endif

// #define RECEIVER_SCREEN 1

// VESC current, for graphs only
const int MOTOR_MIN = -30;
const int MOTOR_MAX = 30;
const int BATTERY_MIN = -30;
const int BATTERY_MAX = 30;

// default board configuration
const int MAX_SPEED = 30;       // KM/H
const int MAX_RANGE = 30;       // KM
const int BATTERY_CELLS = 10;
const int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
const int MOTOR_POLES = 22;
const int WHEEL_DIAMETER = 90;
const int WHEEL_PULLEY = 1;
const int MOTOR_PULLEY = 1;

#define VERSION 1

// Remote > receiver
struct RemotePacket {
  uint32_t address;
  // --------------  // keep 4 byte alignment!
  uint8_t  version;  // 1
  uint8_t  command;	 // Throttle | Light | Settings
  uint8_t  data;     // e.g. throttle value
  uint8_t  crc;
  // --------------  // keep 4 byte alignment!
};

// commands
const uint8_t SET_THROTTLE  = 1;
const uint8_t GET_CONFIG    = 2;
const uint8_t PAIR_REQUEST  = 3;

// Receiver > remote  3 bytes
struct ReceiverPacket {
  uint8_t chain;	      // CRC from RemotePacket
  uint8_t response;
  uint8_t crc;
};

// responses
const uint8_t ACK_ONLY  = 1;
const uint8_t TELEMETRY = 2;
const uint8_t CONFIG    = 3;
const uint8_t PAIRING   = 4;

 // https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences
// https://github.com/cmaglie/FlashStorage

// New VESC values
struct TelemetryPacket {
  uint8_t  chain;	        // CRC from RemotePacket
  // --------------  // keep 4 byte alignment!
  uint16_t voltage;       // volts * 100
  uint16_t speed;         // km/h * 100
  // --------------  // keep 4 byte alignment!
  uint16_t distance;      // km * 100 - 10m accuracy
  int16_t motorCurrent;  // motor amps * 100
  // --------------  // keep 4 byte alignment!
  int16_t inputCurrent;  // battery amps * 100

  uint8_t  crc;	        // CRC

  uint16_t f2w(float f) { return f * 100; } // pack float
  float w2f(uint16_t w) { return float(w) / 100; }; // unpack float

  int16_t f2wi(float f) { return f * 100; } // pack float
  float w2fi(int16_t w) { return float(w) / 100; }; // unpack float

  float getSpeed() { return w2f(speed); }
  void setSpeed(float f) { speed = f2w(f); }

  float getVoltage() { return w2f(voltage); }
  void setVoltage(float f) { voltage = f2w(f); }

  float getDistance() { return w2f(distance); }
  void setDistance(float f) { distance = f2w(f); }

  float getMotorCurrent() { return w2fi(motorCurrent); }
  void setMotorCurrent(float f) { motorCurrent = f2wi(f); }

  float getInputCurrent() { return w2fi(inputCurrent); }
  void setInputCurrent(float f) { inputCurrent = f2wi(f); }
};

// board setting
struct ConfigPacket {
  uint8_t  chain;	        // CRC from RemotePacket
  uint8_t  maxSpeed;	    // m/s
  uint8_t  maxRange;      // km
  uint8_t  batteryCells;
  // --------------  // keep 4 byte alignment!
  uint8_t  batteryType;   // 0: Li-ion | 1: LiPo
  uint8_t  motorPoles;
  uint8_t  wheelDiameter;
  uint8_t  wheelPulley;
  // --------------  // keep 4 byte alignment!
  uint8_t  motorPulley;

  float getMaxSpeed() { return (maxSpeed) / 100; }
  void setMaxSpeed(float f) { maxSpeed = f * 100; }
};

const int default_throttle = 127;

#endif
