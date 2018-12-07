
#include <Arduino.h>
#include "CPU.h"
#include <Adafruit_GFX.h>
#include "globals.h"
#include "radio.h"
#include "utils.h"
#include "VescUart.h"

VescUart UART;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;

// get MAC address / CPU serial
uint32_t boardID;

// send configuration on power on
bool justStarted = true;

// send only updated telemetry
bool telemetryUpdated = false;

// Last time data was pulled from VESC
unsigned long lastUartPull;

// Variables to hold values for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

uint32_t timeoutTimer = 0;
bool receivedData = false;
const short timeoutMax = 500;

// Cruise control
uint16_t cruiseThrottle;
uint16_t cruiseRPM;
bool cruising;

bool connected = false;
uint8_t throttle;

// fonts
#include "fonts/Lato_Regular_7.h"
#include "fonts/Digital.h"
#include "fonts/Pico.h"
#include <Fonts/Org_01.h> // Adafruit

const GFXfont* fontDigital = &Segment13pt7b;  // speed, distance, ...
const GFXfont* fontPico = &Segment6pt7b;      //
const GFXfont* fontDesc = &Dialog_plain_9;    // km/h
const GFXfont* fontMicro = &Org_01;         // connection screen

void acquireSetting();
void calculateRatios();
void controlStatusLed();
void coreTask(void * pvParameters );
bool dataAvailable();
int getSettingValue(uint8_t index);
void getUartData();
bool inRange(int val, int minimum, int maximum);
void loadEEPROMSettings();
void radioExchange();
bool receiveData();
bool sendData(uint8_t response);
void sendUartData();
void setDefaultEEPROMSettings();
void setSettingValue(int index, uint64_t value);
void setStatus(uint8_t code);
void setThrottle(uint16_t value);
void setCruise(uint8_t value);
void speedControl(uint16_t throttle , bool trigger );
String uint64ToAddress(uint64_t number);
String uint64ToString(uint64_t number);
void updateEEPROMSettings();

#ifdef RECEIVER_SCREEN
void updateScreen();
void drawBattery();
#endif

void updateSetting(uint8_t setting, uint64_t value);
