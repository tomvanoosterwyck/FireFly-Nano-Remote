
#include <Arduino.h>
#include "CPU.h"
#include "globals.h"
#include "utils.h"

#include <Adafruit_GFX.h>
#include "Adafruit_SSD1306.h"
#include <Smoothed.h>

#ifdef ARDUINO_SAMD_ZERO

  const int MIN_HALL = 18;
  const int CENTER_HALL = 305;
  const int MAX_HALL = 629;

#elif ESP32

  #include <LoRa.h>
  #include <driver/adc.h>

  // brownout
  #include <driver/rtc_cntl.h>
  #include <driver/rtc_io.h>
  #include <soc/rtc_cntl_reg.h>

  #include <esp_sleep.h>
  #include <esp_deep_sleep.h>

  // flash
  #include <nvs.h>
  #include <nvs_flash.h>
  #include <Preferences.h>

  const int MIN_HALL = 350;
  const int CENTER_HALL = 755;
  const int MAX_HALL = 1023;

  static intr_handle_t s_rtc_isr_handle;

#endif

struct RemoteSettings {
  bool valid;
  bool needSave = false;
  bool debugMode = false;
  short minHallValue = MIN_HALL;
  short centerHallValue = CENTER_HALL;
  short maxHallValue = MAX_HALL;
  uint32_t boardID = 0;
} settings;

RemoteSettings tempSettings;

// calibration
enum calibration_stage {
  CALIBRATE_CENTER,
  CALIBRATE_MAX,
  CALIBRATE_MIN,
  CALIBRATE_STOP
} calibrationStage;

struct alternativeCommand {
  bool needed = false;
  uint8_t command;
  uint8_t data;
} altCmd;

// Data structures
ReceiverPacket recvPacket;
RemotePacket remPacket;
TelemetryPacket telemetry;
ConfigPacket boardConfig;
InfoPacket boardInfo;

bool needConfig = true; // query board confirmation on start

float signalStrength;
float lastRssi;

// Defining struct to hold stats
struct stats {
  float maxSpeed;
  long maxRpm;
  float minVoltage;
  float maxVoltage;
};

enum ui_page {
  PAGE_MAIN,  // speed, battery, distance
  PAGE_EXT,   // current / settings
  PAGE_MENU,
  PAGE_MAX,
  PAGE_DEBUG
} page = PAGE_MAIN;

// Battery monitoring
const float minVoltage = 3.3; // min voltage with vibro motor
const float maxVoltage = 4.2;
const float refVoltage = 3.3; // Feather double-100K resistor divider

unsigned long lastBatterySample = 0; // smooth remote voltage
unsigned long batteryLevelMillis = 0;
bool batteryModeProfileSwitch = true;

// Hall Effect throttle
uint16_t hallValue;
float throttle;

const uint8_t hallNoiseMargin = 8;
byte hallCenterMargin = 0;

AppState state = CONNECTING;
AppState receiverState;

unsigned long stateSwitch = 0;

// OLED display
unsigned long lastSignalBlink;
bool signalBlink = false;
byte counter = 0;
bool displayOn = false;

unsigned long startupTime;

unsigned long lastInteraction; // last time controls were used
unsigned long stopTime;
bool stopped = true;

// Defining variables for radio communication
unsigned long lastTransmission;
short failCount;

unsigned long lastMarker;
unsigned long lastDelay;

// power
bool power = true;
uint8_t shutdownReq = 0;
int batteryLevel = 0; // remote battery

// cruise control
float cruiseSpeed = 0;
int cruiseStartThrottle;
int cruiseThrottle;

bool requestUpdate = false;

// menu
enum menu_page {
  MENU_MAIN,
  MENU_SUB,
  MENU_ITEM,
} menuPage = MENU_MAIN;


// Boardmenu
int currentBoard = 0;
unsigned long alreadySelectedMillis = 0;

const byte mainBoardMenus = 20;

String BOARDMENUS[mainBoardMenus] = {"Board 1", "Board 2", "Board 3", "Board 4", "Board 5", "Board 6", "Board 7", "Board 8", "Board 9", "Board 10", "Board 11", "Board 12", "Board 13", "Board 14", "Board 15", "Board 16", "Board 17", "Board 18", "Board 19", "Board 20"};

enum menu_boards { BOARDS_SELECT, BOARDS_DELETE };

uint32_t boards[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
char* conversionChart[20] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19"};

int selectedBoardSlot = 0;

// Main settings menu
const byte subMenus = 21;
const byte mainMenus = 5;

byte subMenusCount[mainMenus] = {3, 2, 5, 2, 5};

String MENUS[mainMenus][subMenus] = {
    { "Info", "Odometer", "Telemetry", "Debug", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" },
    { "Remote", "Deselect", "Calibrate", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""},
    { "Board", "Turn off", "Info",  "Update", "WiFi On", "WiFi Off", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" },
    { "Mode", "Normal",  "Push", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" },
    { "Profile", "Profile 1",  "Profile 2", "Profile 3", "Profile 4", "Profile 5", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "" }
  };

// TODO: Move Deselect to BOARD

enum menu_main { MENU_INFO, MENU_REMOTE, MENU_BOARD, MENU_MODE, MENU_PROFILE };
enum menu_info { INFO_ODOMETER, INFO_TELEMETRY, INFO_DEBUG };
enum menu_remote { REMOTE_DISCONNECT, REMOTE_CALIBRATE };
enum menu_board { BOARD_OFF, BOARD_INFO, BOARD_UPDATE, BOARD_WIFI_ON, BOARD_WIFI_OFF };
enum menu_mode { MODE_NORMAL, MODE_PUSH };
enum menu_profile { PROFILE_1, PROFILE_2, PROFILE_3, PROFILE_4, PROFILE_5 };


float currentMenu = 0;
int startMenu = 0;
int lowestMenu = 0;
int subMenu = 0;
int subMenuItem = 0;

// set idle mode after using menu
bool menuWasUsed = false;

// const int BATTERY_CELLS = 10;
// const int BATTERY_TYPE = 0;     // 0: LI-ION | 1: LIPO
// const int MOTOR_POLES = 22;
// const int WHEEL_DIAMETER = 90;
// const int WHEEL_PULLEY = 1;
// const int MOTOR_PULLEY = 1;

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
#include <fonts/Org_01_5p.h>

const GFXfont* fontDigital = &Segment13pt7b;  // speed, distance, ...
const GFXfont* fontPico = &Segment6pt7b;      //
const GFXfont* fontDesc = &Dialog_plain_9;    // km/h
const GFXfont* fontMicro = &Org_01_5p;         // connection screen

float getBatteryLevel();
float batteryLevelVolts();
float batteryPackPercentage(float voltage );
void calculateThrottle();
int checkButton();
void checkBatteryLevel();
void coreTask(void * pvParameters );
int cruiseControl();
void drawBatteryLevel();
void drawModeProfile();
void drawConnectingScreen();
void drawMode();
void drawMainPage();
void drawExtPage();
void drawSettingsMenu();
void drawShutdownScreen();
void drawPairingScreen();
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
void loadSettings();
void loop();
bool pressed(int button);
void backToMainMenu();
int readThrottlePosition();
bool receiveData();
bool receivePacket(uint8_t* buf, uint8_t len);
void reset();
bool responseAvailable(uint8_t size);
bool safeCruiseSpeed();
bool sendData();
void setup();
void sleep();
float speed();
bool suddenSpeedChange();
void transmitToReceiver();
bool triggerActive();
bool triggerActiveSafe();
void updateMainDisplay();
void vibrate(int ms);
void drawBoardsMenu();
void loadBoards();
void saveBoard(int board, uint32_t address);
void deleteBoard(int board);
void selectBoard(int board);
void debug(String x);