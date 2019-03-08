
#include "remote.h"

// define display
Adafruit_SSD1306 display(DISPLAY_RST);

Smoothed <double> batterySensor;

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
  #include <FlashStorage.h>

  // Singleton instance of the radio driver
  RH_RF69 radio(RF_CS, RF_DI0);

  FlashStorage(flash_settings, RemoteSettings);

#elif ESP32
  Preferences preferences; // https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences
#endif

#include "radio.h"

void setup() {

  startupTime = millis();

  Serial.begin(115200);

  // while (!Serial) { ; }

  loadSettings();

  #ifdef PIN_VIBRO
    pinMode(PIN_VIBRO, OUTPUT);
  #endif
  #ifdef PIN_BATTERY
    pinMode(PIN_BATTERY, INPUT);
    #ifdef ESP32
     // enable battery probe
      pinMode(VEXT, OUTPUT);
      digitalWrite(VEXT, LOW);
      adcAttachPin(PIN_BATTERY);
      // analogSetClockDiv(255);
    #endif
  #endif

  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);

  #ifdef ARDUINO_SAMD_ZERO // Feather M0 w/Radio
    initRadio(radio);
    // config throttle
    pinMode(PIN_THROTTLE, INPUT);
  #elif ESP32
    initRadio();
    // config throttle
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(ADC_THROTTLE, ADC_ATTEN_DB_2_5);
  #endif

  // 10 seconds average
  batterySensor.begin(SMOOTHED_AVERAGE, 10);

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

  debug("** Esk8-remote transmitter **");

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
  if (millisSince(lastTransmission) < 50) return;

  // mark start
  lastTransmission = millis();

  calculateThrottle();
  transmitToReceiver();
}

void keepAlive() {
  lastInteraction = millis();
}

void calculateThrottle()
{
  int position = readThrottlePosition();

  switch (controlMode) {

  case MODE_IDLE: //
    if (position < default_throttle) { // breaking
      throttle = position; // brakes always enabled
    } else { // throttle >= 0
      if (triggerActive()) {
        // dead man switch activated
        controlMode = MODE_NORMAL;
        throttle = position;
        debug("dead man switch activated");
      } else {
        // locked, ignore
        throttle = default_throttle;
      }
    }
    // sleep timer
    if (stopped && secondsSince(lastInteraction) > REMOTE_SLEEP_TIMEOUT) sleep();
    break;

  case MODE_NORMAL:
    throttle = position;
    debug("MODE_NORMAL");

    // activate cruise mode?
    if (triggerActive() && throttle == default_throttle && speed() > 3) {
      cruiseSpeed = speed();
      // cruiseThrottle = throttle;
      controlMode = MODE_CRUISE;
    }

    // activate deadman switch
    if (stopped && throttle == default_throttle) { // idle
      if (secondsSince(stopTime) > REMOTE_LOCK_TIMEOUT) {
        // lock remote
        controlMode = MODE_IDLE;
        debug("locked");
      }
    }
    break;

  case MODE_MENU: // navigate menu
    // idle
    throttle = default_throttle;

    if (position != default_throttle) {
      menuWasUsed = true;
    }
    break;

  case MODE_ENDLESS: break;

  case MODE_CRUISE:
    // exit mode if trigger released or throttle changed
    if (!triggerActive() || position != default_throttle) {
      controlMode = MODE_NORMAL;
      throttle = position;
      debug("trigger released");
    }
    break;
  }

  // wheel was used
  if (position != default_throttle) keepAlive();
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
    keepAlive();
    // in menu
    if (page == PAGE_MENU) {

      if (menuPage != MENU_MAIN) {
        menuPage = MENU_MAIN; // go back
        currentMenu = 0;
        display.setRotation(DISPLAY_ROTATION); // back to vertical
        calibrationStage = CALIBRATE_CENTER;
        return;
      }
      // exit menu
      controlMode = menuWasUsed ? MODE_IDLE : MODE_NORMAL;
    }

    // switch pages
    page = static_cast<ui_page>((page + 1) % PAGE_MAX);
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

    // keep RTC on
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    gpio_pullup_en((gpio_num_t)PIN_BUTTON);
    gpio_pulldown_dis((gpio_num_t)PIN_BUTTON);

    // wake up when the top button is pressed
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << PIN_BUTTON;
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);

    // turn off radio
    LoRa.end();
    LoRa.sleep();
    delay(100);

    // setup the peripherals state in deep sleep
    pinMode(DISPLAY_SCL, INPUT);
    pinMode(DISPLAY_RST,INPUT);

    pinMode(PIN_VIBRO, INPUT); //

    pinMode(RF_MISO, INPUT);
    pinMode(RF_DI0, INPUT);
    pinMode(RF_MOSI, INPUT);

    // pinMode(5,INPUT);
    // pinMode(14,INPUT);
    // pinMode(18,INPUT);

    // disable battery probe
  	pinMode(VEXT, OUTPUT);
  	digitalWrite(VEXT, HIGH);

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
  #endif

  digitalWrite(LED, HIGH);

  display.powerOn();
  power = true;

  // in case of board change
  needConfig = true;

  keepAlive();
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

void waitRelease(int button) {
  while (true) {
    if (!pressed(button)) return;
  }
}

void loadSettings() {

  #ifdef ESP32
    // esp_err_t err = nvs_flash_init();
    // if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    //   // NVS partition was truncated and needs to be erased
    //   // Retry nvs_flash_init
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   err = nvs_flash_init();
    //   ESP_ERROR_CHECK(err);
    // }

    preferences.begin("FireFlyNano", false);
    settings.minHallValue = preferences.getShort("MIN_HALL",  MIN_HALL);
    settings.centerHallValue = preferences.getShort("CENTER_HALL", CENTER_HALL);
    settings.maxHallValue = preferences.getShort("MAX_HALL", MAX_HALL);
    preferences.end();

  #elif ARDUINO_SAMD_ZERO

    settings = flash_settings.read();

    if (settings.valid == false) {
      settings.minHallValue = MIN_HALL;
      settings.centerHallValue = CENTER_HALL;
      settings.maxHallValue = MAX_HALL;
      debug("defaults loaded");
    }

  #endif

  remPacket.version = VERSION;
}

// only after chamge
void saveSettings() {
  debug("saving settings...");

  #ifdef ESP32

    preferences.begin("FireFlyNano", false);
    preferences.putShort("MIN_HALL",  settings.minHallValue);
    preferences.putShort("CENTER_HALL", settings.centerHallValue);
    preferences.putShort("MAX_HALL", settings.maxHallValue);
    preferences.end();

  #elif ARDUINO_SAMD_ZERO

    settings.valid = true;
    flash_settings.write(settings);

  #endif
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
  bool active = digitalRead(PIN_TRIGGER) == LOW;
  if (active) keepAlive();
  return active;
}

void onTelemetryChanged() {

  switch (controlMode) {

  case MODE_NORMAL: //
  case MODE_IDLE:
    if (telemetry.getSpeed() != 0 || throttle != default_throttle)  {
      // moving
      stopped = false;
    } else {
      if (!stopped) { // just stopped
        stopTime = millis();
        stopped = true;
        debug("stopped");
      }
    }
    break;
  }

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
  byte sz = sizeof(remPacket) + CRC_SIZE; // crc
  uint8_t buf[sz];
  memcpy (buf, &remPacket, sizeof(remPacket));

  // calc crc
  buf[sz-CRC_SIZE] = CRC8(&remPacket, sizeof(remPacket));

  bool sent = false;

  // debug("sending command: " + String(remPacket.command)
  //       + ", data: " + String(remPacket.data)
  //       + ", counter: " + String(remPacket.counter)
  //    );

  #ifdef ESP32

    LoRa.beginPacket(sz);
    int t = 0;
    t = LoRa.write(buf, sz);
    LoRa.endPacket();

    // LoRa.receive(PACKET_SIZE + CRC_SIZE);

    sent = t == sz;

  #elif ARDUINO_SAMD_ZERO

    sent = radio.send(buf, sz);
    if (sent) radio.waitPacketSent();

  #endif

  return sent;
}

bool receiveData() {

  uint8_t len =  PACKET_SIZE + CRC_SIZE;
  uint8_t buf[len];

  // receive a packet and check crc
  if (!receivePacket(buf, len)) return false;

  // parse header
  memcpy(&recvPacket, buf, sizeof(recvPacket));
  if (recvPacket.chain != remPacket.counter) {
    debug("Wrong chain value!");
    return false;
  }

  // response type
  switch (recvPacket.type) {
    case ACK_ONLY:
      // debug("Ack: chain " + String(recvPacket.chain));
      return true;

    case TELEMETRY:
      memcpy(&telemetry, buf, PACKET_SIZE);

      // debug("Telemetry: battery " + String(telemetry.getVoltage())
      //   + ", speed " + String(telemetry.getSpeed())
      //   + ", rpm " + String(telemetry.rpm)
      //   + ", chain " + String(telemetry.header.chain)
      // );

      onTelemetryChanged();

      return true;

    case CONFIG:
      memcpy(&boardConfig, buf, PACKET_SIZE);

      // check chain and CRC
      debug("ConfigPacket: max speed " + String(boardConfig.maxSpeed ));

      needConfig = false;
      return true;
  }

  debug("Unknown response");
  return false;
}

bool receivePacket(uint8_t* buf, uint8_t len) {

  uint8_t expected = len;
  long ms = millis();

  // Should be a message for us now
  if (!responseAvailable(len)) return false;

  #ifdef ARDUINO_SAMD_ZERO

    if (!radio.recv(buf, &len)) return false;

    // signal
    lastRssi = radio.lastRssi();
    signalStrength = constrain(map(lastRssi, -77, -35, 0, 100), 0, 100);

  #elif ESP32
    int i = 0;
    while (LoRa.available()) {
      buf[i] = LoRa.read();
      i++;
    };

    len = i;
    lastRssi = LoRa.packetRssi();
    signalStrength = constrain(map(LoRa.packetRssi(), -100, -50, 0, 100), 0, 100);
  #endif

  // check length
  if (len != expected) {
    debug("Wrong packet length!");
    return false;
  }

  // check crc
  if (CRC8(buf, len - CRC_SIZE) != buf[len - CRC_SIZE]) {
    debug("CRC mismatch!");
    return false;
  }

  return true;
}


bool responseAvailable(uint8_t len) {

  #ifdef ARDUINO_SAMD_ZERO

    return radio.waitAvailableTimeout(REMOTE_RX_TIMEOUT);

  #elif ESP32

    long ms = millis();
    while (true) {
      if (LoRa.parsePacket(len) > 0) return true;
      if (millis() - ms > REMOTE_RX_TIMEOUT) return false; // timeout
    }

  #endif
}

void prepatePacket() {

  if (needConfig) {
    // Ask for board configuration
    remPacket.command = GET_CONFIG;
  } else {
    // speed control
    switch (controlMode) {
    case MODE_CRUISE: // Set cruise mode
      remPacket.command = SET_CRUISE;
      remPacket.data = round(cruiseSpeed);
      break;
    case MODE_IDLE:
    case MODE_NORMAL: // Send throttle to the receiver.
      remPacket.command = SET_THROTTLE;
      remPacket.data = round(throttle);
      break;
    case MODE_MENU:
      remPacket.command = SET_THROTTLE;
      remPacket.data = default_throttle;
      break;
    }
  }
  remPacket.address = boardAddress; // todo: cycle
  remPacket.counter = counter++;
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
      lastDelay = millisSince(lastTransmission);
      digitalWrite(LED, LOW);

      // Transmission was a success (2 seconds after remote startup)
      if (!connected && secondsSince(startupTime) > 2 ) vibrate(200);

      connected = true;
      failCount = 0;
    } else {
      debug("No reply");
      failCount++;
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
    // debug("Disconnected");
  }
}

/*
   Measure the hall sensor output and calculate throttle posistion
*/
int readThrottlePosition() {

  int position = default_throttle;

  // Hall sensor reading can be noisy, lets make an average reading.
  uint32_t total = 0;
  uint8_t samples = 20;

  #ifdef ESP32

    // adc1_config_width(ADC_WIDTH_BIT_10);
    // adc1_config_channel_atten(ADC_THROTTLE, ADC_ATTEN_DB_2_5);

    // // Calculate ADC characteristics i.e. gain and offset factors
    // esp_adc_cal_characteristics_t characteristics;
    // esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_11db, ADC_WIDTH_12Bit, &characteristics);
    //
    // // Read ADC and obtain result in mV
    // uint32_t voltage = adc1_to_voltage(ADC1_CHANNEL_6, &characteristics);
    // printf("%d mV\n",voltage);

    for ( uint8_t i = 0; i < samples; i++ )
    {
      total += adc1_get_raw(ADC_THROTTLE);
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

  // read battery sensor every seconds
  if (secondsSince(lastBatterySample) > 1 || lastBatterySample == 0) {

    lastBatterySample = millis();

    uint16_t total = 0;
    uint8_t samples = 10;

    // read raw value
    for (uint8_t i = 0; i < samples; i++) {
      total += analogRead(PIN_BATTERY);
    }

    // calculate voltage
    float voltage;

    #ifdef ARDUINO_SAMD_ZERO
      voltage = ( (float)total / (float)samples ) * 2 * refVoltage / 1024.0;
    #elif ESP32
      double reading = (double)total / (double)samples;
      voltage = -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
      voltage = voltage * 2.64;
    #endif

    // don't smooth at startup
    if (secondsSince(startupTime) < 3) {
      batterySensor.clear();
    }

    // add to array
    batterySensor.add(voltage);

    lastBatterySample = millis();
  }

  // smoothed value
  return batterySensor.get();
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
      drawMode();
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
  long ms_left = longHoldTime - (millisSince(downTime));
  int w = map(ms_left, 0, longHoldTime - holdTime, 0, 32);
  drawHLine(32 - w, 70, w * 2); // top line
}

void drawConnectingScreen()
{
  display.setRotation(DISPLAY_ROTATION);

  int y = 8;

  display.drawXBitmap((64-24)/2, y, logo, 24, 24, WHITE);

  y += 38;
  drawString("FireFly Nano", -1, y, fontMicro);
  drawString(String(RF_FREQ, 0) + " Mhz", -1, y + 12, fontMicro);

  // blinking icon
  if (millisSince(lastSignalBlink) > 500) {
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
  drawString((triggerActive() ? "T " : "0 ") + String(hallValue) + " " + String(throttle, 0), -1, y, fontMicro);

  // remote battery
  int level = batteryLevel();

  if (level > 20 || signalBlink) { // blink
    y += 12;
    drawString(String(level) + "% " + String(batteryLevelVolts(), 2) + "v", -1, y, fontMicro);
  }

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

void calibrateScreen() {

  int padding = 10;
  int tick = 5;
  int w = display.width() - padding*2;

  int position = readThrottlePosition();

  switch (calibrationStage) {
  case CALIBRATE_CENTER:

    tempSettings.centerHallValue = hallValue;
    tempSettings.minHallValue = hallValue - 100;
    tempSettings.maxHallValue = hallValue + 100;
    calibrationStage = CALIBRATE_MAX;
    break;

  case CALIBRATE_MAX:
    if (hallValue > tempSettings.maxHallValue) {
      tempSettings.maxHallValue = hallValue;
    } else if (hallValue < tempSettings.minHallValue) {
      calibrationStage = CALIBRATE_MIN;
    }
    break;

  case CALIBRATE_MIN:
    if (hallValue < tempSettings.minHallValue) {
      tempSettings.minHallValue = hallValue;
    } else if (hallValue == tempSettings.centerHallValue) {
      calibrationStage = CALIBRATE_STOP;
    }
    break;

  case CALIBRATE_STOP:

    if (pressed(PIN_TRIGGER)) {
      // apply calibration values
      settings.centerHallValue = tempSettings.centerHallValue;
      settings.minHallValue = tempSettings.minHallValue;
      settings.maxHallValue = tempSettings.maxHallValue;

      display.setRotation(DISPLAY_ROTATION);
      menuPage = MENU_MAIN;
      currentMenu = 0;

      saveSettings();
    }

  }

  display.setRotation(DISPLAY_ROTATION_90);

  int center = map(tempSettings.centerHallValue, tempSettings.minHallValue, tempSettings.maxHallValue, display.width() - padding, padding);

  int y = 8;
  drawString(String(tempSettings.maxHallValue), 0, y, fontDesc);
  drawString(String(tempSettings.centerHallValue), center-10, y, fontDesc);
  drawString(String(tempSettings.minHallValue), 128-20, y, fontDesc);

  // line
  y = 16;
  drawHLine(padding, y, w);
  // ticks
  drawVLine(padding, y-tick, tick);
  drawVLine(center, y-tick, tick);
  drawVLine(w + padding, y-tick, tick);

  // current throttle position
  int th = map(hallValue, tempSettings.minHallValue, tempSettings.maxHallValue, display.width() - padding, padding);
  drawVLine(constrain(th, 0, display.width()-1), y, tick);

  y = 32;
  drawString(String(hallValue), constrain(th-10, 0, w), y, fontDesc); // min

  y = 48;
  switch (calibrationStage) {
  case CALIBRATE_MAX:
  case CALIBRATE_MIN:
    drawString("Press throttle fully", -1, y, fontDesc);
    drawString("forward and backward", -1, y+14, fontDesc);
    break;
  case CALIBRATE_STOP:
    drawString("Calibration completed", -1, y, fontDesc);
    drawString("Trigger: Save", -1, y+14, fontDesc);
  }

}

void drawSettingsMenu() {

  //  display.drawFrame(0,0,64,128);
  int y = 10;

  // check speed
  if (controlMode != MODE_MENU) {

    if (telemetry.getSpeed() != 0) {
      drawString("Stop", -1, y=50, fontDesc);
      drawString("to use", -1, y+=14, fontDesc);
      drawString("menu", -1, y+=14, fontDesc);
      return;
    } else { // enable menu
      controlMode = MODE_MENU;
    }
  }

  // wheel = up/down
  int position = readThrottlePosition();

  // todo: wheel control
  if (position < default_throttle - 30) {
    if (currentMenu < subMenus-1) currentMenu += 0.25;
  }
  if (position > default_throttle + 30) {
    if (currentMenu > 0) currentMenu -= 0.25;
  }

  switch (menuPage) {

  case MENU_MAIN:

    drawString("- Menu -", -1, y, fontDesc);

    y += 20;
    for (int i = 0; i < mainMenus; i++) {
      drawString(MENUS[i][0], -1, y, fontDesc);
      // draw cursor
      if (i == round(currentMenu)) drawFrame(0, y-10, 64, 14);
      y += 16;
    }

    if (pressed(PIN_TRIGGER)) {
      menuPage = MENU_SUB;
      subMenu = round(currentMenu);
      currentMenu = 0;
      waitRelease(PIN_TRIGGER);
    }
    break;

  case MENU_SUB:
    // header
    drawString("- " + MENUS[subMenu][0] + " -", -1, y, fontDesc);
    y += 20;
    for (int i = 0; i < subMenus-1; i++) {
      drawString(MENUS[subMenu][i+1], -1, y, fontDesc);
      // draw cursor
      if (i == round(currentMenu)) drawFrame(0, y-10, 64, 14);
      y += 16;
    }

    if (pressed(PIN_TRIGGER)) {
      menuPage = MENU_ITEM;
      subMenuItem = round(currentMenu);
      waitRelease(PIN_TRIGGER);


    }

    break;

  case MENU_ITEM:

    switch (subMenu) {
    case MENU_INFO:
      switch (subMenuItem) {
        case INFO_DEBUG: drawDebugPage(); break;
      }
      break;

    case MENU_REMOTE:
      switch (subMenuItem) {
      case REMOTE_CALIBRATE: calibrateScreen(); break;
      }
      break;

    case MENU_BOARD:

      break;

    }


    break;


  }

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
  drawStringCenter(String(readThrottlePosition()), String(hallValue), y);

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
    x = (display.width() - getStringWidth(string)) / 2;
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
  bars = map(throttle, 0, 254, -10, 10);
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

  uint8_t level = batteryLevel(); // 0 - 100

  // blinking
  if (level < 20) {
    if (millisSince(lastSignalBlink) > 500) {
        signalBlink = !signalBlink;
        lastSignalBlink = millis();
      }
  } else signalBlink = false;

  // blink
  if (signalBlink) return;

  drawFrame(x, y, 18, 9);
  drawBox(x + 18, y + 2, 2, 5);

  // battery level
  drawBox(x + 2, y + 2, level * 14 / 100, 5);

  if (level <= 9) { // < 10%
    drawString(String(level), x + 7, y + 6, fontMicro);
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
