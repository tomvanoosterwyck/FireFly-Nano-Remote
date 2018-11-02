#include <U8g2lib.h>
#include <RH_RF69.h>

// Uncomment DEBUG if you need to debug the remote
//#define DEBUG

#define VERSION 2.0

#ifdef DEBUG
	#define debug(x)  Serial.println (x)
//	#include "printf.h"
#else
	#define debug(x)
#endif

/************ Radio Setup ***************/

// Change to 915.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

// Singleton instance of the radio driver
RH_RF69 radio(RFM69_CS, RFM69_INT);

// Defining the type of display used (128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R3, U8X8_PIN_NONE); 

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

// Button constants
const int CLICK     = 1;
const int DBL_CLICK = 2;
const int HOLD      = 3;
const int LONG_HOLD = 4;

// Button timing variables
int debounce = 20;          // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;            // max ms between clicks for a double click event
int holdTime = 300;        // ms hold period: how long to wait for press+hold event
int longHoldTime = 1500;    // ms long hold period: how long to wait for press+hold event

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

// Defining struct to handle callback data (auto ack)
struct callback {
	float ampHours;
	float inpVoltage;
	long rpm;
	long tachometerAbs;
} returnData;

// Transmit and receive package
struct package {		// | Normal 	| Setting 	| Dummy
	uint8_t type;		// | 0 			| 1			| 2
	uint16_t throttle;	// | Throttle 	| 			| 
	uint8_t trigger;	// | Trigger 	| 			| 
} remPackage;

byte remPackageBuf[sizeof(remPackage)]; // = {0};

// Define package to transmit settings
struct settingPackage {
	uint8_t setting;
	uint64_t value; 
} setPackage;

// Defining struct to hold stats 
struct stats {
	float maxSpeed;
	long maxRpm;
	float minVoltage;
	float maxVoltage;
};

// Defining struct to hold setting values while remote is turned on.
struct settings {
	uint8_t triggerMode; 	// 0
	uint8_t batteryType; 	// 1
	uint8_t batteryCells; 	// 2
	uint8_t motorPoles;		// 3
	uint8_t motorPulley;	// 4
	uint8_t wheelPulley; 	// 5
	uint8_t wheelDiameter;	// 6
	uint8_t controlMode; 	// 7
	short minHallValue; 		// 8
	short centerHallValue;	// 9
	short maxHallValue; 		// 10
	uint64_t address; 		// 11
  float firmVersion; 
} txSettings;

// Defining constants to hold the special settings, so it's easy changed thoughout the code
#define TRIGGER 0
#define MODE    7
#define ADDRESS 11
#define RESET 12
#define EXIT 13

// Defining variables to hold values for speed and distance calculation
float gearRatio;
float ratioRpmSpeed;
float ratioPulseDistance;

float signalStrength;

uint8_t currentSetting = 0;
const uint8_t numOfSettings = 14;

// Setting rules format: default, min, max.
const short rules[numOfSettings][3] {
	{0, 0, 1}, 		// 0: Killswitch 	| 1: Cruise control
	{0, 0, 1}, 		// 0: Li-ion 		  | 1: LiPo
	{10, 0, 12},	// Cell count
	{22, 0, 250},	// Motor poles
	{1, 0, 250},	// Motor pully
	{1, 0, 250},	// Wheel pulley
	{90, 0, 250},	// Wheel diameter
	{1, 0, 2}, 		// 0: PPM only   | 1: PPM and UART | 2: UART only
	{18, 0, 300},	// Min hall value
	{325, 200, 600},	// Center hall value
	{629, 600, 1023},	// Max hall value
	{-1, 0, 0}, 	  // Address
	{-1, 0   , 0}, 	    // Set default address
  {-1, 0, 0}
};

const char titles[numOfSettings][17] = {
  "Trigger use", "Battery type", "Battery cells", "Motor poles", "Motor pulley",
  "Wheel pulley", "Wheel diameter", "Control mode", "Throttle min", "Throttle center",
  "Throttle max", "Generate address", "Reset address", "Settings"
};

const uint8_t unitIdentifier[numOfSettings]  = {0,0,1,0,2,2,3,0,0,0,0,0,0,0};
const uint8_t valueIdentifier[numOfSettings] = {1,2,0,0,0,0,0,3,0,0,0,0,0,0};

const char stringValues[3][3][13] = {
  {"Killswitch", "Cruise", ""},
  {"Li-ion", "LiPo", ""},
  {"PPM", "PPM and UART", "UART only"},
};

const char settingUnits[3][3] = {"S", "T", "mm"};
const char dataSuffix[3][4] = {"KMH", "KM", "%"};
const char dataPrefix[3][9] = {"SPEED", "DISTANCE", "BATTERY"};

// 
bool power = true;

// Pin defination
const int buttonPin = 0;              // RX - pushbutton pin
const uint8_t triggerPin = 10;
const uint8_t batteryMeasurePin = A7; // Feather battery
const uint8_t hallSensorPin = A5;
const uint8_t vibroPin = 6;     

// Battery monitoring
const float minVoltage = 3.1;
const float maxVoltage = 4.2;
const float refVoltage = 3.3; // Feather double-100K resistor divider

// Defining variables for Hall Effect throttle.
uint16_t hallValue, throttle;
const uint8_t hallNoiseMargin = 8;
byte hallCenterMargin = 2;
const uint8_t hallMenuMargin = 100;
uint8_t throttlePosition; 

#define TOP 0
#define MIDDLE 1
#define BOTTOM 2

// Defining variables for radio communication
unsigned long lastTransmission;
bool connected = false;
short failCount;

// Defining variables for OLED display
String tString;
//uint8_t displayData = 0;
uint8_t x, y;
unsigned long lastSignalBlink;
bool signalBlink = false;

unsigned long lastScreenUpdate;

// Defining variables for Settings menu
bool changeSettings     = false; // Global flag for whether or not one is editing the settings
//bool changeThisSetting  = false;
//bool settingsLoopFlag   = false;
bool triggerFlag = false;
bool settingScrollFlag  = false;
bool settingsChangeValueFlag = false;
//unsigned short settingWaitDelay = 500;
//unsigned short settingScrollWait = 800;
//unsigned long settingChangeMillis = 0;
uint8_t shutdownReq = 0;

void setup() {
    
	Serial.begin(115200);
  
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  
  debug("** Esk8-remote receiver **");

  setDefaultEEPROMSettings();
  calculateRatios();
	
	pinMode(triggerPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
	pinMode(hallSensorPin, INPUT);
	pinMode(batteryMeasurePin, INPUT);

  pinMode(LED, OUTPUT);
  pinMode(vibroPin, OUTPUT);

  // Start OLED operations
	u8g2.begin();
 
	// Start radio communication
	initiateTransmitter();
}

void loop() {

	calculateThrottlePosition();

	// Normal transmission. The state of the trigger, cruise and throttle is handled by the receiver. 
	remPackage.type = 0;
  
	remPackage.trigger = triggerActive();
	remPackage.throttle = throttle;
  
  switch (checkButton()) {
  case CLICK: 
    // todo: menu
    break;
  case HOLD: // start shutdown
    vibrate(100);
    break;
  case LONG_HOLD: // shutdown confirmed
    sleep();
    return;
  }
  
  // Transmit to receiver
  transmitToReceiver();
  
  // Call function to update display
  updateMainDisplay();
}

void isr() { } // Interrupt Service Routine

void sleep() 
{  
  if (power == false) { return; }
  
  // turn off screen
  u8g2.setPowerSave(1);  
  power = false;

  // interrupt
  attachInterrupt (digitalPinToInterrupt(buttonPin), isr, LOW);  // attach interrupt handler
  
  // radio
  radio.sleep();

  digitalWrite(LED, LOW);

  USBDevice.standby();
  
  delay(200);

  // Set sleep mode to deep sleep 
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  //Enter sleep mode and wait for interrupt (WFI)
  __DSB();
  __WFI();

  // After waking the code continues
  // to execute from this point.
  
  detachInterrupt(digitalPinToInterrupt(buttonPin));

  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

  USBDevice.attach();

  power = true;
}

void reset() {
  NVIC_SystemReset();
}

bool pressed(int button) {
  return digitalRead(button) == LOW;
}

/*
 * Save the default settings in the EEPROM
 */
void setDefaultEEPROMSettings() {
	for ( uint8_t i = 0; i < numOfSettings; i++ )
	{
		setSettingValue( i, rules[i][0] );
	}

  txSettings.firmVersion = VERSION;
//	txSettings.address = defaultAddress;
//	updateEEPROMSettings();
}

/*
 * Update values used to calculate speed and distance travelled.
 */
void calculateRatios() {
	// Gearing ratio
	gearRatio = (float)txSettings.motorPulley / (float)txSettings.wheelPulley; 
	// ERPM to Km/h
	ratioRpmSpeed = (gearRatio * 60 * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles / 2) * 1000000); 
	// Pulses to km travelled
	ratioPulseDistance = (gearRatio * (float)txSettings.wheelDiameter * 3.14156) / (((float)txSettings.motorPoles * 3) * 1000000); 
}

/* 
 * Get settings value by index (usefull when iterating through settings)
 */
short getSettingValue(uint8_t index) {
	short value;
	switch (index) {
		case TRIGGER: 	value = txSettings.triggerMode; break;
		case 1: 		value = txSettings.batteryType; 	  break;
		case 2: 		value = txSettings.batteryCells; 	  break;
		case 3: 		value = txSettings.motorPoles; 		  break;
		case 4: 		value = txSettings.motorPulley; 	  break;
		case 5: 		value = txSettings.wheelPulley; 	  break;
		case 6: 		value = txSettings.wheelDiameter; 	break;
		case MODE:  value = txSettings.controlMode; 	  break;
		case 8: 		value = txSettings.minHallValue; 	  break;
		case 9: 		value = txSettings.centerHallValue; break;
		case 10: 		value = txSettings.maxHallValue; 	  break;

    default: /* Do nothing */ break;
	}
	return value;
}

/* 
 * Set a value of a specific setting by index.
 */
void setSettingValue(uint8_t index, uint64_t value) {
	switch (index) {
		case TRIGGER: 	txSettings.triggerMode = value; 	  break;
		case 1: 		    txSettings.batteryType = value; 	  break;
		case 2: 		    txSettings.batteryCells = value; 	  break;
		case 3: 		    txSettings.motorPoles = value; 		  break;
		case 4: 		    txSettings.motorPulley = value; 	  break;
		case 5: 		    txSettings.wheelPulley = value; 	  break;
		case 6: 		    txSettings.wheelDiameter = value;	  break;
		case MODE: 		  txSettings.controlMode = value; 	  break;
		case 8: 		    txSettings.minHallValue = value; 	  break;
		case 9: 		    txSettings.centerHallValue = value; break;
		case 10: 		    txSettings.maxHallValue = value; 	  break;
		case ADDRESS: 	txSettings.address = value; 		    break;

    default: /* Do nothing */ break;
	}
}

/*
 * Check if an integer is within a min and max value
 */ 
bool inRange(short val, short minimum, short maximum) {
	return ((minimum <= val) && (val <= maximum));
}

/* 
 * Return true if trigger is activated, false otherwice
 */ 
bool triggerActive() {
	if (digitalRead(triggerPin) == LOW)
		return true;
	else
		return false;
}

/*
 * Return true if trigger is activated with no/low throttle only
 */
bool triggerActiveSafe() {

  bool active = triggerActive();
  if (!active) return false;

  // still on
  if (remPackage.trigger) return true;
    
  // changed (off >> on)
  if (throttle < 150) {
    // low throttle
    return true;
  } else { 
    // unsafe start
    vibrate(60);
    return false;
  }
}

/*
 * Function used to transmit the remPackage and receive auto acknowledgement.
 */
void transmitToReceiver(){

	// Transmit once every 50 millisecond
	if ( millis() - lastTransmission >= 50 ) {

		lastTransmission = millis();

		// Transmit the remPackage
    byte sz=sizeof(remPackage);
    memcpy (remPackageBuf, &remPackage, sz);
    
		if ( radio.send(remPackageBuf, sz  ))
		{
      radio.waitPacketSent();
      digitalWrite(LED, HIGH);

			// Listen for an acknowledgement reponse (return of uart data).
      if (radio.available()) { // Should be a message for us now  

        uint8_t buf[sizeof(returnData)];
        uint8_t len = sizeof(buf);
        
        if (radio.recv(buf, &len)) {

          memcpy(&returnData, buf, sizeof(returnData));

          digitalWrite(LED, LOW);

          // Transmission was a success          
          if (!connected) vibrate(200);  

          connected = true;
          failCount = 0;    

          // signal
          signalStrength = constrain(map(radio.lastRssi(), -77, -35, 0, 100), 0, 100);
        }
        
      } else {
        debug("No reply");        
        failCount++;
        signalStrength = 0;
      }
			
		} else {
			// Transmission was not a succes
			failCount++;

			debug("Failed transmission");
		}

		// If lost more than 5 transmissions, we can assume that connection is lost.
		if (failCount > 5) {

      if (connected) vibrate(200);
      
			connected = false;
      
//      debug("Disconnected");
		}
	}
}

/*
 * Initiate the radio module
 */
void initiateTransmitter(){

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
if (!radio.init()) {
    debug("RFM69 radio init failed");
    while (1);
  }
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!radio.setFrequency(RF69_FREQ)) {
    debug("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  radio.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  // Replace dots with random bytes from this page:
  // https://www.random.org/cgi-bin/randbyte?nbytes=16&format=h
  uint8_t key[] = { 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x..,
                    0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x..};
                    
  radio.setEncryptionKey(key);
  
  debug(String(RF69_FREQ) + " Mhz");  
}

/*
 * Update the OLED for each loop
 * To-Do: Only update display when needed
 */
void updateMainDisplay()
{
  if (power == false) return;
  
  u8g2.setPowerSave(0); // check?
	
 
		if ( changeSettings == false )
		{
      if (isShuttingDown()) {
    
        drawShutdownScreen(); 
      
      } else if (connected) {

        // 1s update

        if (millis() - lastScreenUpdate > 250) {
          u8g2.clearBuffer();

          drawPage();
          drawBatteryLevel();
          drawSignal();    
          
          //drawThrottle();
      
          lastScreenUpdate = millis();
        }

        u8g2.sendBuffer();
        
        
      } else {
        
        u8g2.firstPage();
        do {
          drawThrottle();
          drawConnectingScreen();

        } while ( u8g2.nextPage() );
        
      }
      
		} else {
      // drawSettingsMenu();
    }
}


/*
 * Measure the hall sensor output and calculate throttle posistion
 */
void calculateThrottlePosition()
{
	// Hall sensor reading can be noisy, lets make an average reading.
	uint16_t total = 0;
  uint8_t samples = 20;

	for ( uint8_t i = 0; i < samples; i++ )
	{
		total += analogRead(hallSensorPin);
	}

	hallValue = total / samples;
	
	debug(hallValue);

  if (hallValue >= txSettings.centerHallValue + hallNoiseMargin) {
    throttle = constrain(map(hallValue, txSettings.centerHallValue + hallNoiseMargin, txSettings.maxHallValue, 127, 255), 127, 255);
  } 
  else if (hallValue <= txSettings.centerHallValue - hallNoiseMargin) {
    throttle = constrain(map(hallValue, txSettings.minHallValue, txSettings.centerHallValue - hallNoiseMargin, 0, 127), 0, 127);
  }
  else {
    // Default value if stick is in deadzone
    throttle = 127;
  }

  // removeing center noise
  if (abs(throttle - 127) < hallCenterMargin) {
    throttle = 127;
  }

}

/* 
 * Calculate the remotes battery voltage
 */ 
float batteryLevelVolt() {

  uint16_t total = 0;
  uint8_t samples = 5;

  for (uint8_t i = 0; i < samples; i++) {
    total += analogRead(batteryMeasurePin);
  }

  return ( (float)total / (float)samples ) * 2 * refVoltage / 1024.0;
}
  
/* 
 * Calculate the remotes battery level
 */ 
float batteryLevel() {

  float voltage = batteryLevelVolt();
  
  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  } 
  
  return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
}


/*
 * Calculate the battery level of the board based on the telemetry voltage
 */
float batteryPackPercentage( float voltage ){

	float maxCellVoltage = 4.2;
	float minCellVoltage;

	if(txSettings.batteryType == 0){
		// Li-ion
		minCellVoltage = 3.1; 
	}
	else
	{
		// Li-po
		minCellVoltage = 3.4;
	}

	float percentage = (100 - ( (maxCellVoltage - voltage / txSettings.batteryCells)/((maxCellVoltage - minCellVoltage)) ) * 100);

	if(percentage > 100.0){
		return 100.0;  
	}else if (percentage < 0.0){
		return 0.0;
	}
	
	return percentage;
}


void drawShutdownScreen() 
{
  u8g2.firstPage();
  do {      
    drawString("Turning off...", -1, 60, u8g2_font_crox1h_tf);
  
    // shrinking line
    long ms_left = longHoldTime - (millis() - downTime);
    int w = map(ms_left, 0, longHoldTime - holdTime, 0, 32);
    u8g2.drawHLine(32 - w, 70, w * 2); // top line

  } while ( u8g2.nextPage() );
}

void drawConnectingScreen() 
{     
    int y = 8;
    
    u8g2.drawXBMP((64-24)/2, y, 24, 24, logo);
  
    drawString("Firefly Nano", -1, y + 42, u8g2_font_crox1h_tf);
 
    drawString(String(RF69_FREQ, 0) + " Mhz", -1, y + 42 + 14, u8g2_font_blipfest_07_tr);
  
    // remote battery
    //drawString(String(batteryLevel(), 0) + " - " + String(throttle), -1, y + 42 + 28, u8g2_font_blipfest_07_tr);
  
    drawString(String(hallValue) + " - " + String(throttle) + " - " + String(batteryLevel(), 0), -1, y + 42 + 28, u8g2_font_blipfest_07_tr);
  
  
    if (millis() - lastSignalBlink > 500) {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }
    y = 88;
    if (signalBlink == true) {
      u8g2.drawXBMP((64-12)/2, y, 12, 12, connectedIcon);
    } else {
      u8g2.drawXBMP((64-12)/2, y, 12, 12, noconnectionIcon);
    }
  
    drawString("CONNECTING...", -1, 116, u8g2_font_blipfest_07_tr); 
  
}

void drawThrottle() {

  if (throttle > 127) {
    // right side - throttle
    int h = map(throttle - 127, 0, 127, 0, 128);
    u8g2.drawVLine(63, 128-h, h); // nose    
  }

  if (throttle < 127) {
    // left side - brake
    int h = map(throttle, 0, 127, 128, 0);
    u8g2.drawVLine(0, 0, h); // nose    
  }  
}

/*
 * Print the main page: Throttle, battery level and telemetry
 */
void drawPage() {

  uint8_t decimals;
  float value;
  uint16_t first, last;

  String s;
  
  uint8_t offset = 38;
  x = 0;
  y = 37;
  uint8_t width;

//  u8g2.drawFrame(0,0,64,128);
  
  // --- Speed ---
  value = ratioRpmSpeed * abs(returnData.rpm);
  float speedMax = 30.0;

  drawStringCenter(String(value, 0), "km/h", y);

  y = 48;
  // speedometer graph height array
  uint8_t a[16] = {3, 3, 4, 4, 5, 6, 7, 8, 10, 
    11, 13, 15, 17, 20, 24, 28};
  uint8_t h;
  
  for (uint8_t i = 0; i < 16; i++) {
    h = a[i];
    if (speedMax / 16 * i <= value) {
      u8g2.drawVLine(x + i*4 + 2, y - h, h);
    } else {
      u8g2.drawPixel(x + i*4 + 2, y - h);
      u8g2.drawPixel(x + i*4 + 2, y - 1);
    }
  }
  
  // --- Battery ---
  value = batteryPackPercentage( returnData.inpVoltage );

  y = 73;
  
  int battery = (int) value;
  drawStringCenter(String(battery), "%", y);

  drawString(String(returnData.inpVoltage, 1), 50, 73, u8g2_font_blipfest_07_tr);

  y = 78;
  x = 1;

  // longboard body
  h = 12;
  uint8_t w = 41;
  u8g2.drawHLine(x + 10, y, w); // top line
  u8g2.drawHLine(x + 10, y + h, w); // bottom

  // nose
  u8g2.drawHLine(x + 2, y + 3, 5); // top line
  u8g2.drawHLine(x + 2, y + h - 3, 5); // bottom
  
  u8g2.drawPixel(x + 1, y + 4); 
  u8g2.drawVLine(x, y + 5, 3); // nose
  u8g2.drawPixel(x + 1, y + h - 4); 

  u8g2.drawLine(x + 6, y + 3, x + 9, y);          // / 
  u8g2.drawLine(x + 6, y + h - 3, x + 9, y + h);  // \

  // tail
  u8g2.drawHLine(64 - 6 - 2, y + 3, 5); // top line
  u8g2.drawHLine(64 - 6 - 2, y + h - 3, 5); // bottom

  u8g2.drawPixel(64 - 3, y + 4); 
  u8g2.drawVLine(64 - 2, y + 5, 3); // tail
  u8g2.drawPixel(64 - 3, y + h - 4); 

  u8g2.drawLine(64 - 6 - 3, y + 3, 64 - 6 - 6, y);          // / 
  u8g2.drawLine(64 - 6 - 3, y + h - 3, 64 - 6 - 6, y + h);  // \
 
  // longboard wheels
  u8g2.drawBox(x + 3, y, 3, 2); // left
  u8g2.drawBox(x + 3, y + h - 1, 3, 2);
  u8g2.drawBox(64 - 7, y, 3, 2); // right
  u8g2.drawBox(64 - 7, y + h - 1, 3, 2);
  
  // battery sections
  for (uint8_t i = 0; i < 14; i++) {
    if (round((100 / 14) * i) <= value) {
      u8g2.drawBox(x + i*3 + 10, y + 2, 1, h - 3);
    }
  }

  // --- Distance in km ---
  value = ratioPulseDistance * returnData.tachometerAbs;
  String km;

  y = 118;

  if (value >= 1) {
    km = String(value, 0);  
    drawStringCenter(String(km), "km", y);
  } else {
    km = String(value * 1000, 0);
    drawStringCenter(String(km), "m", y);
  }

  // max distance
  int range = 30;
  if (value > range) range = value;
  
  drawString(String(range), 56, 118, u8g2_font_blipfest_07_tr); // u8g2_font_prospero_bold_nbp_tn

  // dots
  y = 122;
  for (uint8_t i = 0; i < 16; i++) {
    u8g2.drawBox(x + i * 4, y + 4, 2, 2);
  }

  // start end
  u8g2.drawBox(x, y, 2, 6);
  u8g2.drawBox(62, y, 2, 6);
  u8g2.drawBox(30, y, 2, 6);

  // position
  u8g2.drawBox(x, y + 2, value / range * 62, 4);
}
 
void drawStringCenter(String value, String caption, uint8_t y){

  static char cache[10];

  // draw digits
  int x = 0;
  value.toCharArray(cache, value.length() + 1);
  u8g2.setFont(u8g2_font_ncenB18_tn); //u8g2_font_t0_18b_tr);
  u8g2.drawStr(x, y, cache);

  // draw caption km/%
  x += u8g2.getStrWidth(cache) + 4;
  y -= 9;
  caption.toCharArray(cache, caption.length() + 1);
  u8g2.setFont(u8g2_font_crox1h_tf);
  u8g2.drawStr(x, y, cache);
}

void drawString(String string, int x, int y, const uint8_t *font){

  static char cache[20];
  string.toCharArray(cache, string.length() + 1);
  u8g2.setFont(font); 

  if (x == -1) {
    x = (64 - u8g2.getStrWidth(cache)) / 2;
  }

  u8g2.drawStr(x, y, cache);
}

/*
 * Print the signal icon if connected, and flash the icon if not connected
 */
void drawSignal() {

  x = 45; 
  y = 11;

  for (int i = 0; i < 9; i++) {
    if (round((100 / 9) * i) <= signalStrength) 
      u8g2.drawVLine(x + (2 * i), y - i, i);
  }
}

/*
 * Print the remotes battery level as a battery on the OLED
 */
void drawBatteryLevel() {

  x = 2; 
  y = 2;

  uint8_t level = batteryLevel();

  u8g2.drawFrame(x, y, 18, 9);
  u8g2.drawBox(x + 18, y + 2, 2, 5);

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 2 + (3 * i), y + 2, 2, 5);
    }
  }
}

int checkButton() {    
  
   int event = 0;
   buttonVal = digitalRead(buttonPin);
   
   // Button pressed down
   if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       holdEventPast = false;
       longHoldEventPast = false;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
   }
   // Button released
   else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false) DCwaiting = true;
           else
           {
               event = DBL_CLICK;
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }
   // Test for normal click event: DCgap expired
   if ( buttonVal == HIGH && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
   {
       event = CLICK;
       DCwaiting = false;
   }
   // Test for hold
   if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
       // Trigger "normal" hold
       if (not holdEventPast)
       {
           event = HOLD;
           waitForUp = true;
           ignoreUp = true;
           DConUp = false;
           DCwaiting = false;
           holdEventPast = true;
       }
       // Trigger "long" hold
       if ((millis() - downTime) >= longHoldTime)
       {
           if (not longHoldEventPast)
           {
               event = LONG_HOLD;
               longHoldEventPast = true;
           }
       }
   }
   buttonLast = buttonVal;
   return event;
}

bool isShuttingDown() {
  // button held for more than holdTime
  return (buttonVal == LOW) && holdEventPast;
}

void vibrate(int ms) {
  digitalWrite(vibroPin, HIGH);
  delay(ms);
  digitalWrite(vibroPin, LOW);
}
