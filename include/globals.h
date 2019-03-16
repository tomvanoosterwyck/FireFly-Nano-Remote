#ifndef _CONST_H
#define _CONST_H

#include <Arduino.h>
#include <datatypes.h>

#define DEBUG // Uncomment DEBUG if you need to debug the remote

// #define FAKE_UART // Uncomment if VESC not connected

// const COMM_PACKET_ID VESC_COMMAND = COMM_GET_VALUES; // VESC
const COMM_PACKET_ID VESC_COMMAND = COMM_GET_UNITY_VALUES; // Enertion Unity

/*
  Connect receiver and open Serial Monitor (Cmd+Shift+M),
  it will display the correct chip ID.
*/
// const uint32_t boardAddress = 0xc89cb368; // Feather
// const uint32_t boardAddress = 0x88bf713c; // TTGO
// const uint32_t boardAddress = 0xfea4ae30; // TTGO w/o screen
const uint32_t boardAddress = 0x8cbf713c; // Heltec

// Auto stop (in seconds)
const float AUTO_BRAKE_TIME = 5.0;      // time to apply the full break
const int AUTO_BRAKE_RELEASE = 5;       // time to release breaks after the full stop

// UART
const int UART_SPEED = 115200;
const uint16_t uartPullInterval = 150;
const int UART_TIMEOUT = 10; // 10ms for 115200 bauds, 100ms for 9600 bauds
const int REMOTE_RX_TIMEOUT = 20; // ms

const int REMOTE_LOCK_TIMEOUT = 10; // seconds to lock throttle when idle
const int REMOTE_SLEEP_TIMEOUT = 180; // seconds to go to sleep mode

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
  uint8_t  counter;
  // --------------
};

// commands
const uint8_t SET_THROTTLE  = 1;
const uint8_t SET_CRUISE    = 2;

const uint8_t GET_CONFIG    = 3;
const uint8_t SET_STATE     = 4;

// state machine
enum BoardState {
  IDLE,       // remote is not connected
  PUSHING,
  AUTO_CRUISE,
  CONNECTED,  // riding with connected remote
  STOPPING,   // emergency brake when remote has disconnected
  STOPPED,    //
  UPDATE      // update over WiFi
};

// Receiver > remote  3 bytes
struct ReceiverPacket {
  uint8_t type;
  uint8_t chain;	// CRC from RemotePacket
  uint8_t state;   // Mode: Pairing, BT, ...
  uint8_t r2;
};

// responses type
const uint8_t ACK_ONLY  = 1;
const uint8_t TELEMETRY = 2;
const uint8_t CONFIG    = 3;
const uint8_t PAIRING   = 4;

struct AckPacket {
  ReceiverPacket header;
  // --------------  // keep 4 byte alignment!
  uint8_t r0;
  uint8_t r1;        // UART delay?
  uint16_t r2;
  // --------------
  uint16_t r3;       // temperature?
  uint16_t r4;
  // --------------
  int16_t r5;
  int16_t r6;
  // --------------
};

const int PACKET_SIZE = sizeof(AckPacket);
const int CRC_SIZE = 1;

// New VESC values 12 + 3
struct TelemetryPacket {
  ReceiverPacket header;
  // -----------------  // keep 4 byte alignment!
  uint16_t speed;       // km/h * 100
  uint8_t tempMotor;
  uint8_t tempFET;
  // -----------------
  uint16_t voltage;     // volts * 100
  uint16_t distance;    // km * 100 - 10m accuracy
  // -----------------
  int16_t motorCurrent; // motor amps * 100
  int16_t inputCurrent; // battery amps * 100
  // -----------------

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
  ReceiverPacket header;
  // -------------------  // keep 4 byte alignment!
  uint8_t  maxSpeed;	    // m/s
  uint8_t  maxRange;      // km
  uint8_t  batteryCells;
  uint8_t  batteryType;   // 0: Li-ion | 1: LiPo
  // -------------------
  uint8_t  motorPoles;
  uint8_t  wheelDiameter;
  uint8_t  wheelPulley;
  uint8_t  motorPulley;
  // -------------------
  int16_t r1;  // battery amps * 100
  int16_t r2;
  // -------------------
  float getMaxSpeed() { return (maxSpeed) / 100; }
  void setMaxSpeed(float f) { maxSpeed = f * 100; }
};

const int default_throttle = 127;

#ifdef DEBUG
  #define debug(x) Serial.println (x)
#else
  #define debug(x)
#endif

#endif
