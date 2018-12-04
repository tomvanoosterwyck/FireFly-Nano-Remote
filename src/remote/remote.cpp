
#include "remote.h"

// define display
Adafruit_SSD1306 display(DISPLAY_RST);

// Adafruit_SSD1306 display(64, 128, &Wire, DISPLAY_RST, 700000, 700000);

/************ Radio Setup ***************/
#define drawVLine(x, y, l) display.drawLine (x, y, x, y + l, WHITE); // display.drawVerticalLine (x, y, l);
#define drawHLine(x, y, l)  display.drawLine (x, y, x + l, y, WHITE);
#define drawBox(x, y, w, h) display.fillRect(x, y, w, h, WHITE);
#define drawFrame(x, y, w, h) display.drawRect(x, y, w, h, WHITE);
#define drawPixel(x, y)  display.drawPixel (x, y, WHITE);
#define drawStr(x, y, s) display.drawString(x, y, s);

// Feather M0 w/Radio
#ifdef ARDUINO_SAMD_ZERO // Feather M0 w/Radio

  #include <RH_RF69.h>

  // Singleton instance of the radio driver
  RH_RF69 radio(RF_CS, RF_DI0);

#endif

#include "radio.h"

// Defining struct to hold stats
struct stats {
  float maxSpeed;
  long maxRpm;
  float minVoltage;
  float maxVoltage;
};


// const char titles[numOfSettings][17] = {
//   "Trigger use", "Battery type", "Battery cells", "Motor poles", "Motor pulley",
//   "Wheel pulley", "Wheel diameter", "Control mode", "Throttle min", "Throttle center",
//   "Throttle max", "Generate address", "Reset address", "Settings"
// };

// const uint8_t unitIdentifier[numOfSettings]  = {0, 0, 1, 0, 2, 2, 3, 0, 0, 0, 0, 0, 0, 0};
// const uint8_t valueIdentifier[numOfSettings] = {1, 2, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0};

// const char stringValues[3][3][13] = {
//   {"Killswitch", "Cruise", ""},
//   {"Li-ion", "LiPo", ""},
//   {"PPM", "PPM and UART", "UART only"},
// };
//
// const char settingUnits[3][3] = {"S", "T", "mm"};
// const char dataSuffix[3][4] = {"KMH", "KM", "%"};
// const char dataPrefix[3][9] = {"SPEED", "DISTANCE", "BATTERY"};

//
bool power = true;

const uint8_t hallNoiseMargin = 8;
byte hallCenterMargin = 0;
const uint8_t hallMenuMargin = 100;
uint8_t throttlePosition;

#define TOP 0
#define MIDDLE 1
#define BOTTOM 2

// Defining variables for radio communication
unsigned long lastTransmission;
bool connected = false;
short failCount;

unsigned long lastMarker;
unsigned long lastDelay;

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

float cruiseSpeed = 0;
int cruiseStartThrottle;
int cruiseThrottle;

void setup() {
  Serial.begin(115200);

  // while (!Serial) { ; }

  setDefaults();

  #ifdef PIN_VIBRO
    pinMode(PIN_VIBRO, OUTPUT);
  #endif
  #ifdef PIN_BATTERY
    pinMode(PIN_BATTERY, INPUT);
  #endif

  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);

  // Start radio communication
  #ifdef ARDUINO_SAMD_ZERO // Feather M0 w/Radio
    initRadio(radio);
  #elif ESP32
    initRadio();
  #endif

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(DISPLAY_ROTATION);
  display.powerOn();

  #ifdef ESP32
    xTaskCreatePinnedToCore(
      coreTask,   /* Function to implement the task */
      "coreTask", /* Name of the task */
      10000,      /* Stack size in words */
      NULL,       /* Task input parameter */
      configMAX_PRIORITIES - 1,  /* Priority of the task. 0 is slower */
      NULL,       /* Task handle. */
      0);  /* Core where the task should run */
  #endif

  debug("** Esk8-remote receiver **");

}

#ifdef ESP32 // core 0
  void coreTask( void * pvParameters ) {
    while (1) {
      radioLoop();
      vTaskDelay(1);
    }
  }
#endif

void loop() { // core 1

  #ifdef ARDUINO_SAMD_ZERO
    radioLoop();
  #endif

  handleButtons();

  // Call function to update display
  updateMainDisplay();
}

void radioLoop() {
  // Transmit once every 50 millisecond
  if (millis() - lastTransmission < 50) return;

  // mark start
  lastTransmission = millis();

  calculateThrottle();
  transmitToReceiver();
}

void calculateThrottle()
{
  int position = readThrottlePosition();

  // braking
  if (position <= default_throttle) {
    throttle = position;
    return;
  }

  // simple deadman switch
  throttle = triggerActive() ? position : default_throttle;
  return;


//  if (!connected)

  switch (controlMode) {

  case MODE_IDLE: //

    throttle = default_throttle;

    // dead man switch activated
    if (triggerActive()) {
      controlMode = MODE_NORMAL;
      throttle = position;

      debug("dead man switch activated");
    }
    break;

  case MODE_NORMAL:

    throttle = position;

    // if (triggerActive() && safeCruiseSpeed()) {
    //   cruiseSpeed = speed();
    //   cruiseThrottle = throttle;
    //   controlMode = MODE_CRUISE;
    //
    // }
    debug("MODE_NORMAL");
    break;

  case MODE_STOP: // reset required

    throttle = default_throttle; // keep cruise mode
    break;

  case MODE_ENDLESS:
    break;

  case MODE_CRUISE:

    // if (triggerActive()) {
    //
    //   if (position == default_throttle) { // cruising
    //     // adjust throttle
    //     throttle = cruiseControl();
    //
    //   } else {
    //     throttle = position; // keep cruise mode
    //   }
    //
    // } else { // trigger released
    //
    //   controlMode = MODE_NORMAL;
    //   throttle = position;
    //   debug("trigger released");
    //
    // }
    break;

  }

}

// int cruiseControl() {
//
//   if (speed() == 0) return throttle;
//
//   debug("cruise @" + String(cruiseSpeed));
//
//   // speed difference
//   float diff = cruiseSpeed - speed(); // 10 kmh - 5 kmh = 5 km/h
//
//   debug("speed: " + String(speed()) + ", diff: " + String(diff));
//
// }


void isr() { } // Interrupt Service Routine

void handleButtons() {

  switch (checkButton()) {
    case CLICK:
      // todo: menu
      page = static_cast<ui_page>((page + 1) % PAGE_MAX);

      // skip menu
      debug("click");

      break;
    case HOLD: // start shutdown
      vibrate(100);
      break;
    case LONG_HOLD: // shutdown confirmed
      sleep();
      return;
  }

}

void sleep()
{
  if (power == false) { return; }

  // turn off screen
  display.powerOff();
  digitalWrite(LED, LOW);

  power = false;

  #ifdef ARDUINO_SAMD_ZERO

    // interrupt
    attachInterrupt (digitalPinToInterrupt(PIN_BUTTON), isr, LOW);  // attach interrupt handler

    radio.sleep();


    USBDevice.standby();

    delay(200);

    // Set sleep mode to deep sleep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    //Enter sleep mode and wait for interrupt (WFI)
    __DSB();
    __WFI();

  #elif ESP32

    // wait for button release
    while (pressed(PIN_BUTTON)) vTaskDelay(10);

    esp_deep_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);

    // Enter sleep mode and wait for interrupt
    esp_deep_sleep_start();

    // CPU will be reset here
  #endif

  // After waking the code continues
  // to execute from this point.

  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON));

  #ifdef ARDUINO_SAMD_ZERO
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    USBDevice.attach();
  #elif ESP32


  #endif

  digitalWrite(LED, HIGH);

  display.powerOn();
  power = true;

  // in case of board change
  needConfig = true;
}

/*
void sleep2()
{
  if (power == false) {
    return;
  }

  // turn off screen
  display.powerOff();
  power = false;

  // interrupt
  attachInterrupt (digitalPinToInterrupt(PIN_BUTTON), isr, LOW);  // attach interrupt handler

  digitalWrite(LED, LOW);

  CPU::sleep(PIN_BUTTON);

  // After waking the code continues
  // to execute from this point.

  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON));
  digitalWrite(LED, HIGH);
  power = true;

}
*/

bool pressed(int button) {
  return digitalRead(button) == LOW;
}

/*
   Save the default settings in the EEPROM
*/
void setDefaults() {

  remPacket.version = VERSION;

  settings.minHallValue = MIN_HALL;
  settings.centerHallValue = CENTER_HALL;
  settings.maxHallValue = MAX_HALL;

}

/*
   Check if an integer is within a min and max value
*/
bool inRange(short val, short minimum, short maximum) {
  return ((minimum <= val) && (val <= maximum));
}

/*
   Return true if trigger is activated, false otherwice
*/
bool triggerActive() {
  return digitalRead(PIN_TRIGGER) == LOW;
}

/*
   Return true if trigger is activated with no/low throttle only

bool triggerActiveSafe() {

  bool active = triggerActive();
  if (!active) return false;

  // still on
  if (remPacket.trigger) return true;

  // changed (off >> on)
  if (throttle < 150) {
    // low throttle
    return true;
  } else {
    // unsafe start
    vibrate(60);
    return false;
  }
} */

bool sendData() {

  // Transmit the remPacket
  byte sz = sizeof(remPacket);
  uint8_t buf[sz];
  memcpy (buf, &remPacket, sz);

  bool sent = false;

  debug("sending command:" + String(remPacket.command)
      + ", crc: " + String(remPacket.crc, HEX));

  #ifdef ESP32

    LoRa.beginPacket(sz);

    int t = 0;

    t = LoRa.write(buf, sz);

    LoRa.endPacket();

    //LoRa.receive(sizeof(ReceiverPacket));

    sent = t > 0;

  #elif ARDUINO_SAMD_ZERO

    sent = radio.send(buf, sz);
    if (sent) radio.waitPacketSent();

  #endif

  // debug("sent");

  return sent;
}

bool receiveData() {

  uint8_t len;
  uint8_t buf[sizeof(TelemetryPacket)]; // biggest packet

  // receive ACK packet
  len = sizeof(ReceiverPacket);

  if (!receivePacket(buf, len)) return false;

  memcpy(&recvPacket, buf, len);

  // check dynamic code
  if (recvPacket.chain != remPacket.crc) return false;
  // check recvPacket.crc

  debug("Response: " + String(recvPacket.response)
    + ", chain: " + String(recvPacket.chain)
    + ", CRC: " + String(recvPacket.crc));

  // any extra data?
  switch (recvPacket.response) {

  case ACK_ONLY: return true;

  case TELEMETRY: // receive telemetry

    len = sizeof(TelemetryPacket);
    if (receivePacket(buf, len)) {

        memcpy(&telemetry, buf, len);

        // check chain and CRC

        debug("battery " + String(telemetry.getVoltage()));
        return true;
    }
    break;

  case CONFIG: // receive board configuration

    len = sizeof(ConfigPacket);
    if (receivePacket(buf, len)) {

        memcpy(&boardConfig, buf, len);

        // check chain and CRC
        debug("max speed " + String(boardConfig.maxSpeed ));

        needConfig = false;
        return true;
    }
    break;

  }

  return false;
}

bool receivePacket(uint8_t* buf, uint8_t len) {

  // Should be a message for us now
  if (!responseAvailable(len)) return false;

  debug("receive packet ");

  #ifdef ARDUINO_SAMD_ZERO

    bool received = radio.recv(buf, &len);

    if (received) {
      //memcpy(&returnData, buf, sizeof(returnData));

      // signal
      signalStrength = constrain(map(radio.lastRssi(), -77, -35, 0, 100), 0, 100);
    }

    return received;

  #elif ESP32

    int i = 0;
    while (LoRa.available()) {
      buf[i] = LoRa.read();
      i++;
      // check length


    }
    //memcpy(&returnData, buf, len);

    //    debug("rc "+String(returnData.inpVoltage));

    lastRssi = LoRa.packetRssi();

    signalStrength = constrain(map(LoRa.packetRssi(), -100, -50, 0, 100), 0, 100);

    return true;

  #endif
}


bool responseAvailable(uint8_t size) {

  long ms = millis();

  #ifdef ARDUINO_SAMD_ZERO

    while (true) {

      if (radio.available()) return true;
      // wait 10ms
      if (millis() - ms > 10) return false; // timeout
    }

  #elif ESP32

    while (true) {
      // data available?
      if (LoRa.parsePacket(size) > 0) return true;
      // wait 10ms
      if (millis() - ms > 30) return false; // timeout
    }

  #endif
}

void prepatePacket() {

  if (needConfig) {
    // Ask for board configuration
    remPacket.command = GET_CONFIG;
  } else {
    // Normal transmission. Send throttle to the receiver.
    remPacket.command = SET_THROTTLE;
    remPacket.data = round(throttle);
  }
  remPacket.address = boardAddress; // cycle
  remPacket.crc = CRC8(&remPacket, sizeof(remPacket)-1);
}
/*
   Function used to transmit the remPacket and receive auto acknowledgement.
*/
void transmitToReceiver() {

  // send packet
  digitalWrite(LED, HIGH);

  prepatePacket();

  if (sendData()) {

    // Listen for an acknowledgement reponse and return of uart data
    if (receiveData()) {

      // transmission time
      lastDelay = millis() - lastTransmission;

      digitalWrite(LED, LOW);

      // Transmission was a success
      if (!connected) vibrate(200);

      connected = true;
      failCount = 0;

    } else {

      debug("No reply");
      failCount++;

      // repeat now
      //lastTransmission = 0;
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

/*
   Measure the hall sensor output and calculate throttle posistion
*/
int readThrottlePosition() {

  int position = default_throttle;

  // Hall sensor reading can be noisy, lets make an average reading.
  uint32_t total = 0;
  uint8_t samples = 32;

  #ifdef ESP32

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC1_GPIO34_CHANNEL, ADC_ATTEN_DB_2_5);

    for ( uint8_t i = 0; i < samples; i++ )
    {
      total += adc1_get_raw(ADC1_GPIO34_CHANNEL);
    }

  #elif ARDUINO_SAMD_ZERO

    for ( uint8_t i = 0; i < samples; i++ )
    {
      total += analogRead(PIN_THROTTLE);
    }

  #endif

  hallValue = total / samples;
  // debug(hallValue);

  // map values 0..1023 >> 0..255
  if (hallValue >= settings.centerHallValue + hallNoiseMargin) {    // 127 > 150
    position = constrain(map(hallValue, settings.centerHallValue + hallNoiseMargin, settings.maxHallValue, 127, 255), 127, 255);
  }
  else if (hallValue <= settings.centerHallValue - hallNoiseMargin) {
    position = constrain(map(hallValue, settings.minHallValue, settings.centerHallValue - hallNoiseMargin, 0, 127), 0, 127);
  }
  else {
    // Default value if stick is in deadzone
    position = default_throttle;
  }

  // removeing center noise, todo: percent
  if (abs(throttle - default_throttle) < hallCenterMargin) {
    position = default_throttle;
  }

  return position;
}

/*
   Calculate the remotes battery voltage
*/
float batteryLevelVolts() {

  uint16_t total = 0;
  uint8_t samples = 5;

  #ifdef ARDUINO_SAMD_ZERO

    for (uint8_t i = 0; i < samples; i++) {
      total += analogRead(PIN_BATTERY);
    }

    return ( (float)total / (float)samples ) * 2 * refVoltage / 1024.0;

  #elif ESP32

    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_DB_11);

    for ( uint8_t i = 0; i < samples; i++ )
    {
      total += adc1_get_raw(ADC1_GPIO35_CHANNEL);
    }
    // check!
    return ( (float)total / (float)samples ) / 1024.0 * 2 * refVoltage * 1.1;

  #endif

}

/*
   Calculate the remotes battery level
*/
float batteryLevel() {

  float voltage = batteryLevelVolts();

  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  }

  return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
}


/*
   Calculate the battery level of the board based on the telemetry voltage
*/
float batteryPackPercentage( float voltage ) {

  float maxCellVoltage = 4.2;
  float minCellVoltage;

  if (boardConfig.batteryType == 0) { // Li-ion
    minCellVoltage = 3.1;
  } else { // Li-po
    minCellVoltage = 3.4;
  }

  float percentage = (100 - ( (maxCellVoltage - voltage / boardConfig.batteryCells) / ((maxCellVoltage - minCellVoltage)) ) * 100);

  if (percentage > 100.0) {
    return 100.0;
  } else if (percentage < 0.0) {
    return 0.0;
  }

  return percentage;
}

// --------------------------------

/*
   Update the OLED for each loop
*/
void updateMainDisplay()
{
  display.clearDisplay();
  display.setTextColor(WHITE);

  if (isShuttingDown()) drawShutdownScreen();

  else if (connected) { // main UI

     // 10 ms


    switch (page) {
    case PAGE_MAIN:
      drawBatteryLevel(); // 2 ms
      // drawMode();
      drawSignal(); // 1 ms
      drawMainPage();
      break;
    case PAGE_EXT:  drawExtPage(); break;
    case PAGE_MENU: drawSettingsMenu(); break;
    case PAGE_DEBUG: drawDebugPage(); break;
    }

  } else { // connecting
    drawConnectingScreen();
    drawThrottle();
  }

  display.display();
}

void drawShutdownScreen()
{
  drawString("Turning off...", -1, 60, fontMicro);

  // shrinking line
  long ms_left = longHoldTime - (millis() - downTime);
  int w = map(ms_left, 0, longHoldTime - holdTime, 0, 32);
  drawHLine(32 - w, 70, w * 2); // top line
}

void drawConnectingScreen()
{
  int y = 8;

  display.drawXBitmap((64-24)/2, y, logo, 24, 24, WHITE);

  y += 38;
  drawString("FireFly Nano", -1, y, fontMicro);
  drawString(String(RF_FREQ, 0) + " Mhz", -1, y + 12, fontMicro);

  // blinking icon
  if (millis() - lastSignalBlink > 500) {
    signalBlink = !signalBlink;
    lastSignalBlink = millis();
  }

  y = 68;
  if (signalBlink == true) {
    display.drawXBitmap((64-12)/2, y, connectedIcon, 12, 12, WHITE);
  } else {
    display.drawXBitmap((64-12)/2, y, noconnectionIcon, 12, 12, WHITE);
  }

  y += 12 + 12;
  drawString("Connecting...", -1, y, fontMicro);

  // hall + throttle
  y += 14;
  String tr = "0";
  if (triggerActive()) tr = "T";
  drawString(tr + " " + String(hallValue) + " " + String(round(throttle)), -1, y, fontMicro);

  // remote battery
  y += 12;
  drawString(String(batteryLevel(), 0) + "% " + String(batteryLevelVolts(), 2) + "v", -1, y, fontMicro);

}

void drawThrottle() {

  if (throttle > 127) {
    // right side - throttle
    int h = map(throttle - 127, 0, 127, 0, 128);
    drawVLine(63, 128 - h, h); // nose
  }

  if (throttle < 127) {
    // left side - brake
    int h = map(throttle, 0, 127, 128, 0);
    drawVLine(0, 0, h); // nose
  }
}

void drawSettingsMenu() {

  //  display.drawFrame(0,0,64,128);

  int y = 10;
  drawString("Menu", -1, y, fontDesc);

}

void drawDebugPage() {

  //  display.drawFrame(0,0,64,128);

  int y = 10;
  drawString("Debug", -1, y, fontDesc);

  y = 35;
  if (connected) drawStringCenter(String(lastDelay), " ms", y);
  else drawStringCenter("", "No reply", y);

  y += 25;
  if (connected) drawStringCenter(String(lastRssi, 0), " db", y);

  y += 25;
  drawStringCenter(String(throttle, 0), String(hallValue), y);

}

int getStringWidth(String s) {

  int16_t x1, y1;
  uint16_t w1, h1;

  display.getTextBounds(s, 0, 0, &x1, &y1, &w1, &h1);
  return w1;
}

void drawString(String string, int x, int y, const GFXfont *font) {

  display.setFont(font);

  if (x == -1) {
    x = (64 - getStringWidth(string)) / 2;
  }

  display.setCursor(x, y);
  display.print(string);
}

float speed() {
  return telemetry.getSpeed();
}

void drawMode() {

  String m = "?";

  switch (controlMode) {

  case MODE_IDLE:
    m = "!";
    break;

  case MODE_NORMAL:
    m = "N";
    break;

  case MODE_STOP:
    m = "S";
    break;

  case MODE_ENDLESS:
    m = "E";
    break;

  case MODE_CRUISE:
    m = "C";
    break;
  }

  // top center
  drawString(m, -1, 10, fontPico);

}

void drawBars(int x, int y, int bars, String caption, String s) {

  const int width = 14;

  drawString(caption, x + 4, 10, fontDesc);

  if (bars > 0) { // up
    for (int i = 0; i <= 10; i++)
      if (i <= bars) drawHLine(x, y - i*3, width);
  } else { // down
    for (int i = 0; i >= -10; i--)
      if (i >= bars) drawHLine(x, y - i*3, width);
  }

  // frame
  drawHLine(x, y-33, width);
  drawHLine(x, y+33, width);

  // values
  drawString(s, x, y + 48, fontPico);
}

/*
   Print the main page: Throttle, battery level and telemetry
*/
void drawExtPage() {

  const int gap = 20;

  int x = 5;
  int y = 48;
  float value;
  int bars;

  drawHLine(2, y, 64-2);

  // 1 - throttle
  value = throttle;  //telemetry.getInputCurrent
  bars = map(throttle, 0, 255, -10, 10);
  drawBars(x, y, bars, "T", String(bars));

  // motor current
  x += gap;
  bars = map(telemetry.getMotorCurrent(), MOTOR_MIN, MOTOR_MAX, -10, 10);
  drawBars(x, y, bars, "M", String(telemetry.getMotorCurrent(),0));

  // battery current
  x += gap;
  bars = map(telemetry.getInputCurrent(), BATTERY_MIN, BATTERY_MAX, -10, 10);
  drawBars(x, y, bars, "B", String(telemetry.getInputCurrent(), 0) );

}

/*
   Print the main page: Throttle, battery level and telemetry
*/
void drawMainPage() {

  float value;

  String s;

  int x = 0;
  int y = 37;

  //  display.drawFrame(0,0,64,128);

  // --- Speed ---
  value = speed();
  float speedMax = boardConfig.maxSpeed;

  String m;

  if (controlMode == MODE_CRUISE) m = String(cruiseSpeed);
  else m = "km/h";

  drawStringCenter(String(value, 0), m, y);

  y = 48;
  // speedometer graph height array
  uint8_t a[16] = {3, 3, 4, 4, 5, 6, 7, 8, 10,
                   11, 13, 15, 17, 20, 24, 28
                  };
  uint8_t h;

  for (uint8_t i = 0; i < 16; i++) {
    h = a[i];
    if (speedMax / 16 * i <= value) {
      drawVLine(x + i * 4 + 2, y - h, h);
    } else {
      drawPixel(x + i * 4 + 2, y - h);
      drawPixel(x + i * 4 + 2, y - 1);
    }
  }

  // --- Battery ---
  value = batteryPackPercentage( telemetry.getVoltage() );

  y = 74;

  int battery = (int) value;
  drawStringCenter(String(battery), "%", y);

  drawString(String(telemetry.getVoltage(), 1), 44, 73, fontPico);

  y = 80;
  x = 1;

  // longboard body
  h = 12;
  uint8_t w = 41;
  drawHLine(x + 10, y, w); // top line
  drawHLine(x + 10, y + h, w); // bottom

  // nose
  drawHLine(x + 2, y + 3, 5); // top line
  drawHLine(x + 2, y + h - 3, 5); // bottom

  drawPixel(x + 1, y + 4);
  drawVLine(x, y + 5, 3); // nose
  drawPixel(x + 1, y + h - 4);

  display.drawLine(x + 6, y + 3, x + 9, y, WHITE);
  display.drawLine(x + 6, y + h - 3, x + 9, y + h, WHITE);

  // tail
  drawHLine(64 - 6 - 2, y + 3, 5); // top line
  drawHLine(64 - 6 - 2, y + h - 3, 5); // bottom

  drawPixel(64 - 3, y + 4);
  drawVLine(64 - 2, y + 5, 3); // tail
  drawPixel(64 - 3, y + h - 4);

  display.drawLine(64 - 6 - 3, y + 3, 64 - 6 - 6, y, WHITE);
  display.drawLine(64 - 6 - 3, y + h - 3, 64 - 6 - 6, y + h, WHITE);

  // longboard wheels
  drawBox(x + 3, y, 3, 2); // left
  drawBox(x + 3, y + h - 1, 3, 2);
  drawBox(64 - 7, y, 3, 2); // right
  drawBox(64 - 7, y + h - 1, 3, 2);

  // battery sections
  for (uint8_t i = 0; i < 14; i++) {
    if (round((100 / 14) * i) <= value) {
      drawBox(x + i * 3 + 10, y + 2, 1, h - 3);
    }
  }

  // --- Distance in km ---
  value = telemetry.getDistance();
  String km;

  y = 118;

  if (value >= 1) {
    km = String(value, 0);
    drawStringCenter(km, "km", y);
  } else {
    km = String(value * 1000, 0);
    drawStringCenter(km, "m", y);
  }

  // max distance
  int range = 30;
  if (value > range) range = value;

  drawString(String(range), 52, 118, fontPico);

  // dots
  y = 122;
  for (uint8_t i = 0; i < 16; i++) {
    drawBox(x + i * 4, y + 4, 2, 2);
  }

  // start end
  drawBox(x, y, 2, 6);
  drawBox(62, y, 2, 6);
  drawBox(30, y, 2, 6);

  // position
  drawBox(x, y + 2, value / range * 62, 4);
}

void drawStringCenter(String value, String caption, uint8_t y) {

  // draw digits
  int x = 0;

  display.setFont(fontDigital);

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text

  display.setCursor(x, y);
  display.print(value);

  // draw caption km/%
  x += getStringWidth(value) + 4;
  y -= 9;

  display.setCursor(x, y);
  display.setFont(fontDesc);

  display.print(caption);

}

/*
   Print the signal icon if connected, and flash the icon if not connected
*/
void drawSignal() {

  int x = 45;
  int y = 11;

  for (int i = 0; i < 9; i++) {
    if (round((100 / 9) * i) <= signalStrength)
      drawVLine(x + (2 * i), y - i, i);
  }
}


/*
   Print the remotes battery level as a battery on the OLED
*/
void drawBatteryLevel() {

  int x = 2;
  int y = 2;

  uint8_t level = batteryLevel();

  drawFrame(x, y, 18, 9);
  drawBox(x + 18, y + 2, 2, 5);

  for (uint8_t i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= level)
    {
      drawBox(x + 2 + (3 * i), y + 2, 2, 5);
    }
  }
}

int checkButton() {

  int event = 0;
  buttonVal = digitalRead(PIN_BUTTON);

  // Button pressed down
  if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
  {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
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
  if ( buttonVal == HIGH && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
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
  #ifdef PIN_VIBRO
    digitalWrite(PIN_VIBRO, HIGH);
    delay(ms);
    digitalWrite(PIN_VIBRO, LOW);
  #endif
}
