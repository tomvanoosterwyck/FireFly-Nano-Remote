
#include <Arduino.h>
#include "CPU.h"
#include "globals.h"
#include "utils.h"
// #include <Preferences.h>

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"

#ifdef ARDUINO_SAMD_ZERO
  #include <RH_RF69.h>
#elif ESP32
  #include <LoRa.h>
  #include <driver/adc.h>
  #include <esp_sleep.h>
  #include <esp_deep_sleep.h>
#endif

struct RemoteSettings {
  short minHallValue; 		// 8
  short centerHallValue;	// 9
  short maxHallValue; 		// 10
} settings;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;

bool needConfig = true; // query board confirmation on start

float signalStrength;
float lastRssi;

enum ui_page {
  PAGE_MAIN,  // speed, battery, distance
  PAGE_EXT,   // current / settings
  PAGE_MENU,
  PAGE_DEBUG,
  PAGE_MAX
} page = PAGE_MAIN;

// speed control
enum control_mode {
  MODE_IDLE,
  MODE_NORMAL,
  MODE_CRUISE,
  MODE_ENDLESS,
  MODE_STOP
} controlMode = MODE_IDLE;

// Battery monitoring
const float minVoltage = 3.1;
const float maxVoltage = 4.2;
const float refVoltage = 3.3; // Feather double-100K resistor divider

// Defining variables for Hall Effect throttle.
uint16_t hallValue;
float throttle;

// Defining variables for OLED display
unsigned long lastSignalBlink;
bool signalBlink = false;
byte counter = 0;

// Button constants
const int CLICK     = 1;
const int DBL_CLICK = 2;
const int HOLD      = 3;
const int LONG_HOLD = 4;

// Button timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 300;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 1000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
long downTime = -1;         // time the button was pressed down
long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

// icons
const unsigned char logo[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x80, 0x3c, 0x01,
  0xe0, 0x00, 0x07, 0x70, 0x18, 0x0e, 0x30, 0x18, 0x0c, 0x98, 0x99, 0x19,
  0x80, 0xff, 0x01, 0x04, 0xc3, 0x20, 0x0c, 0x99, 0x30, 0xec, 0xa5, 0x37,
  0xec, 0xa5, 0x37, 0x0c, 0x99, 0x30, 0x04, 0xc3, 0x20, 0x80, 0xff, 0x01,
  0x98, 0x99, 0x19, 0x30, 0x18, 0x0c, 0x70, 0x18, 0x0e, 0xe0, 0x00, 0x07,
  0x80, 0x3c, 0x01, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char transmittingIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char connectedIcon[] PROGMEM = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char noconnectionIcon[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// fonts
#include "fonts/Lato_Regular_7.h"
#include "fonts/Digital.h"
#include "fonts/Pico.h"
#include <Fonts/Org_01.h> // Adafruit

const GFXfont* fontDigital = &Segment13pt7b;  // speed, distance, ...
const GFXfont* fontPico = &Segment6pt7b;      //
const GFXfont* fontDesc = &Dialog_plain_9;    // km/h
const GFXfont* fontMicro = &Org_01;         // connection screen

float batteryLevel();
float batteryLevelVolts();
float batteryPackPercentage(float voltage );
void calculateThrottle();
int checkButton();
void coreTask(void * pvParameters );
int cruiseControl();
void drawBatteryLevel();
void drawConnectingScreen();
void drawMode();
void drawMainPage();
void drawExtPage();
void drawSettingsMenu();
void drawShutdownScreen();
void drawDebugPage();
void drawSignal();
void drawStringCenter(String value, String caption, uint8_t y);
void drawString(String string, int x, int y, const GFXfont *font);
void drawThrottle();
int getStringWidth(String s);
void handleButtons();
void initTransmitter();
void radioLoop();
bool inRange(short val, short minimum, short maximum);
void isr();
bool isShuttingDown();
void loop();
bool pressed(int button);
int readThrottlePosition();
bool receiveData();
bool receivePacket(uint8_t* buf, uint8_t len);
void reset();
bool responseAvailable(uint8_t size);
bool safeCruiseSpeed();
bool sendData();
void setDefaults();
void setup();
void sleep();
float speed();
bool suddenSpeedChange();
void transmitToReceiver();
bool triggerActive();
bool triggerActiveSafe();
void updateMainDisplay();
void vibrate(int ms);
