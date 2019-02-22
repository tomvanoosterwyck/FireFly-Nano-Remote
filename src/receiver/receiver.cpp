#include "receiver.h"

#ifdef ARDUINO_SAMD_FEATHER_M0 // Feather M0 w/Radio
  #include <RH_RF69.h>

  // Singleton instance of the radio driver
  RH_RF69 radio(RF_CS, RF_DI0);

#elif ESP32
  #include <LoRa.h>

  // Uart serial
  HardwareSerial MySerial(1);
#endif

#ifdef RECEIVER_SCREEN
  #include <Adafruit_GFX.h>
  #include "Adafruit_SSD1306.h" // local file for TTGO
  Adafruit_SSD1306 display(DISPLAY_RST);
#endif

#include "radio.h"

float signalStrength;
float lastRssi;

// Status blink LED
uint8_t statusCode = 0;
bool statusLedState = false;

unsigned long statusCycleTime, previousStatusMillis, currentMillis, startCycleMillis = 0;

unsigned long lastDelay;

// Initiate VescUart class for UART communication

void setup()
{
  // wait for VESC?
  delay(1000);

  Serial.begin(115200);

  // while (!Serial) {}; // wait for serial port to connect. Needed for native USB port only

  debug("Receiver");

  //loadEEPROMSettings();
  setDefaultEEPROMSettings();

  calculateRatios();

  pinMode(LED, OUTPUT);

  UART.setTimeout(UART_TIMEOUT);

  #ifdef ARDUINO_SAMD_FEATHER_M0

    #ifndef FAKE_UART
      UART.setSerialPort(&Serial1);
      Serial1.begin(UART_SPEED);
    #endif

    initRadio(radio);

  #elif ESP32

    #ifndef FAKE_UART
      UART.setSerialPort(&MySerial);
      MySerial.begin(UART_SPEED, SERIAL_8N1, RX, TX);
    #endif

    initRadio();

    xTaskCreatePinnedToCore(
          coreTask,   /* Function to implement the task */
          "coreTask", /* Name of the task */
          10000,      /* Stack size in words */
          NULL,       /* Task input parameter */
          configMAX_PRIORITIES - 1,          /* Priority of the task */
          NULL,       /* Task handle. */
          0);         /* Core where the task should run */

  #endif

  debug("Setup complete - begin listening");

  #ifdef RECEIVER_SCREEN
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  #endif
}

int getStringWidth(String s) {

  int16_t x1, y1;
  uint16_t w1, h1;

  display.getTextBounds(s, 0, 0, &x1, &y1, &w1, &h1);
  return w1;
}

void drawString(String string, int x, int y, const GFXfont *font) {

  display.setFont(font);

  display.setCursor(x, y);
  display.print(string);
}

void drawStringCentered(String string, int x, int y, const GFXfont *font) {
  display.setFont(font);
  x = x - getStringWidth(string) / 2;
  drawString(string, x, y, font);
}

String getState() {
  switch (state) {
    case IDLE: return "Idle";
    case CONNECTED: return "Normal";  // dB?
    case STOPPING: return "Stopping";
    case STOPPED: return "Stopped";
    case ENDLESS: return "Auto";
    case AUTO_WALK: return "Cruise 5";
    case AUTO_CRUISE: return "Cruise 15";
  }
}

#ifdef RECEIVER_SCREEN
void updateScreen() {

  display.clearDisplay();

  switch (state) {

    case IDLE:
      drawBattery();
      break;

    case AUTO_WALK:
    case AUTO_CRUISE:
      display.setTextColor(WHITE);
      display.setFont(fontDigital);

      display.setCursor(0, 20);
      display.println("CRUISE: 5");
      display.println("SPD: " + String(telemetry.getSpeed(),1));
      break;

    default:
      display.setTextColor(WHITE);
      display.setFont(fontDigital);

      display.setCursor(0, 20);
      display.print("THR: " + String(map(throttle, 0, 255, -100, 100)) + "%");
      display.print("SPD: " + String(telemetry.getSpeed(),1) + " k");
  }

  // ---- status ----
  display.setTextColor(WHITE);

  drawStringCentered(
    getState() + "  " +
    String(telemetry.getVoltage(), 1) + "v  " +
    String(telemetry.getDistance(), 1) + "km",
    64, 62, fontDesc);

  display.display();
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


void drawBattery() {

  display.setTextColor(WHITE);


  if (telemetry.getVoltage() == 0) {

    // no uart connection
    display.setFont();
    display.setCursor(0, 10);
    display.print("No UART data");

    // remote info
    display.setCursor(0, 25);
    if (!connected)
      display.print("Remote not connected");
    else
      display.print("Signal: " + String(lastRssi, 0) + " dB");

//    display.setCursor(0, 40);
//    display.print("Delay: " + String(lastDelay));

    return;
  }

  // --- battery ----
  int w = 120; int h = 46;
  display.drawRect(0, 0, w, h, WHITE);
  display.drawRect(1, 1, w-2, h-2, WHITE);

  display.fillRect(w, 10, 6, h-10-10, WHITE);

  // fill
  float pc = batteryPackPercentage(telemetry.getVoltage());
  int x = w * pc / 100-8;
  display.fillRect(4, 4, x, h-8, WHITE);

  // % value
  if (pc > 50) { // left side
    display.setTextColor(BLACK);
    drawStringCentered(String(pc, 0) + "%", x/2 + 4, 31, fontBig);
  } else { // right side
    display.setTextColor(WHITE);
    drawStringCentered(String(pc, 0) + "%", x + (w - x) / 2, 31, fontBig);
  }

}
#endif // RECEIVER_SCREEN

#ifdef BLUETOOTH
  class MyServerCallbacks: public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("*** connected ******");

      };

      void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
      }
  };

  // receive BLE data
  class MyCallbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {

        std::string rxValue = pCharacteristic->getValue();

        uint8_t messageReceived[256];
        int count = 0;

        if (rxValue.length() > 0) {
          // Serial.println("*********");
          // Serial.print("Received Value: ");
          for (int i = 0; i < rxValue.length(); i++) {
            // line1 += String(rxValue[i], HEX) + " ";
            messageReceived[i] = rxValue[i];
            count++;
          }
          // Serial.print(line1);

          // Forward package to UART
          // stop byte?
          // count++;
          // messageReceived[count] = '\0';

        	MySerial.write(messageReceived, count);
        }
      }
  };

  void stopBLE() {

    // Stop advertising
    pServer->getAdvertising()->stop();

    // Start the service
    pService->stop();


  }

  void startBLE() {

    // Create the BLE Device
    BLEDevice::init("FireFly Nano Receiver");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
  				  CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
  					CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    debug("Waiting a client connection to notify...");

  }
#endif

void loop() // core 1
{
  // get telemetry");
  if (!deviceConnected) getUartData(); // every 250  ms

  #ifdef ARDUINO_SAMD_ZERO
    radioExchange();
    stateMachine();
  #elif RECEIVER_SCREEN
    updateScreen(); // 25 ms
    vTaskDelay(1);
  #endif

  #ifdef BLUETOOTH

  if (deviceConnected) {

    // get response
    if (MySerial.available()) {

      uint8_t payload[256];
      size_t sz = UART.receiveUartMessageRaw(payload);

      // for (int i = 0; i < sz; i++) {
      //   line3 += String(payload[i], HEX) + " ";
      // }

      // send response
      pTxCharacteristic->setValue(payload, sz);
      pTxCharacteristic->notify();
    }

    delay(1); // bluetooth stack will go into congestion, if too many packets are sent
  }
  #endif

}

#ifdef ESP32
  void coreTask( void * pvParameters ){  // core 0

    String taskMessage = "radio task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);

    while (true) {
      radioExchange();
      stateMachine();
      vTaskDelay(1);
    }
  }
#endif

bool receiveData() {
  // debug("receiveData");

  uint8_t len = sizeof(RemotePacket) + CRC_SIZE; // 9
  uint8_t buf[len];

  bool received = false;

  #ifdef ARDUINO_SAMD_ZERO

    received = radio.recv(buf, &len);

  #elif ESP32

    int bytes = 0;
    for (int i = 0; i < len; i++) {
      buf[i] = LoRa.read();
      bytes++;
    }
    // lastRssi = LoRa.packetRssi();
    len = bytes;
    received = true;

  #endif

  if (!received) return false;

  // size check
  if (len != sizeof(RemotePacket) + CRC_SIZE) {
    debug("Wrong packet size");
    return false;
  }

  // crc check
  if (CRC8(buf, sizeof(remPacket)) != buf[sizeof(remPacket)]) {
    debug("CRC mismatch");
    return false;
  }

  // process packet
  memcpy(&remPacket, buf, sizeof(remPacket));

  // address check
  if (remPacket.address != boardID) {
    Serial.print("Wrong Board ID, please use: 0x");
    Serial.println(String(boardID, HEX));
    return false;
  }

  if (remPacket.version != VERSION) {
    Serial.print("Version mismatch!");
  }

  return true;

    // signalStrength = constrain(map(radio.lastRssi(), -77, -35, 0, 100), 0, 100);
}

bool sendPacket(const void * packet, uint8_t len) {

  // calc crc
  uint8_t crc = CRC8(packet, len);

  // struct to buffer
  len += CRC_SIZE;
  uint8_t buf[len];
  memcpy (buf, packet, len);
  buf[len - CRC_SIZE] = crc;

  bool sent = false;

  #ifdef ESP32

    LoRa.beginPacket(len);
    int t = LoRa.write(buf, len);
    LoRa.endPacket();

    sent = t == len;
    // LoRa.receive(sizeof(remPacket) + CRC_SIZE);

  #elif ARDUINO_SAMD_ZERO

    sent = radio.send(buf, len);

    if (sent) radio.waitPacketSent();

  #endif

  return sent;
}

bool sendData(uint8_t response) {

  // send packet
  switch (response) {

  case ACK_ONLY: // no extra data to send
    debug("Sending ack only");
    telemetry.header.type = response;
    telemetry.header.chain = remPacket.counter;
    return sendPacket(&telemetry, sizeof(telemetry));

  case TELEMETRY:
    debug("Sending telemetry");
    telemetry.header.type = response;
    telemetry.header.chain = remPacket.counter;

    if (sendPacket(&telemetry, sizeof(telemetry))) {
      telemetryUpdated = false;
      return true;
    }
    break;

  case CONFIG:
    debug("Sending board configuration");
    boardConfig.header.type = response;
    boardConfig.header.chain = remPacket.counter;

    if (sendPacket(&boardConfig, sizeof(boardConfig))) {
      justStarted = false; // send config once
      return true;
    }
    break;
  }

  return false;
}

bool dataAvailable() {

  #ifdef ARDUINO_SAMD_ZERO

    return radio.available();

  #elif ESP32

    int packetSize = LoRa.parsePacket(sizeof(remPacket) + CRC_SIZE);
    return packetSize > 0;

  #endif
}

bool isNormalState() {
  return state != STOPPING;
}

void radioExchange() {

  // controlStatusLed();

  /* Begin listen for transmission */
  if ( dataAvailable() ) {

    // led on
    digitalWrite(LED, HIGH);

    if (receiveData()) {

      timeoutTimer = millis();
      receivedData = true;
      connected = true;

      debug( "New package: command " + String(remPacket.command) + ", data " + String(remPacket.data) + ", counter " + String(remPacket.counter) );

      // Send acknowledgement
      uint8_t response = ACK_ONLY;

      switch (remPacket.command) {
        case SET_THROTTLE:
          if (isNormalState) { // recover from auto stop
            state = CONNECTED;
          } // else response = AUTO_STOP

          // falltrough
        case SET_CRUISE:
          if (telemetryUpdated) { response = TELEMETRY; }
          break;
        case GET_CONFIG: response = CONFIG; break;
      }

      // send config after power on
      if (justStarted) response = CONFIG;

      // // allow remote to receive
      // delay(5);

      if (sendData(response)) {
        // led on
        digitalWrite(LED, LOW);
        // debug("Sent response");
      }

      // control
      switch (remPacket.command) {
        case SET_THROTTLE: // control VESC speed
          if (isNormalState()) setThrottle(remPacket.data);
          break;

        case SET_CRUISE:
          setCruise(remPacket.data);
          break;
      }

    } else receivedData = false;

  }
  /* End listen for transmission */

}

void autoCruise(uint8_t speed) {

  if (millisSince(lastCruiseControl) > 50) {
    lastCruiseControl = millis();

    setCruise(speed);
  }


}


void stateMachine() {

  switch (state) {

    case IDLE: // no remote connected

      debug("IDLE");
      if (telemetry.getSpeed() >= 5) {
        state = AUTO_WALK;
      }
      break;

    case AUTO_WALK: // cruise without remote at 5 km/h

      autoCruise(5);

      if (telemetry.getSpeed() >= 15) { // NFS
        state = AUTO_CRUISE;
        debug("NFS");
      } else if (telemetry.getSpeed() < 3) { // rider want to stop
        state = IDLE;
        debug("< 3 mh/h");
      }

      // check amps/speed !

      break;

    case AUTO_CRUISE: // cruise without remote at 15 km/h

      debug("AUTO_CRUISE");

      autoCruise(15);

      if (telemetry.getSpeed() < 10) { // rider want to stop
        state = IDLE;
      }

      // check amps/speed !

      break;

    case CONNECTED: // remote is connected

      // timeout handling
      if (millisSince(timeoutTimer) > timeoutMax) {
        debug("receiver timeout");

        // No speed is received within the timeout limit.
        // setStatus(TIMEOUT);
        connected = false;
        state = STOPPING;
        lastBrakeTime = millis();

        // use last throttle
        throttle = lastThrottle;

        timeoutTimer = millis();

        // falltrough

      } else break;

    case STOPPING: // emergency brake when remote has disconnected

      // stop acceleration
      if (throttle > default_throttle) {
        throttle = default_throttle;
        // wait?
      }

      if (secondsSince(lastBrakeTime) > AUTO_BRAKE_INTERVAL) {

        // decrease throttle to brake  127 / 5 * 0.1
        float brakeForce = constrain(default_throttle / AUTO_BRAKE_TIME * AUTO_BRAKE_INTERVAL, 0, 10);

        // apply brakes
        if (throttle > brakeForce) throttle -= brakeForce; else throttle = 0;
        setThrottle(throttle);

        debug("braking...");
        debug(String(throttle,2));

        lastBrakeTime = millis();
      }

      // check speed
      if (throttle == 0 && telemetry.getSpeed() == 0) {
        state = STOPPED;
        lastBrakeTime = millis();
      }

      break;

    case STOPPED:

      if (secondsSince(lastBrakeTime) > AUTO_BRAKE_RELEASE) {
        state = IDLE;
      }
      break;
  }
}


void setStatus(uint8_t code){

  short cycle = 0;

  // switch(code){
  //   case COMPLETE:  cycle = 500;    break;
  //   case FAILED:    cycle = 1400;   break;
  // }
  //
  // currentMillis = millis();
  //
  // if(currentMillis - startCycleMillis >= statusCycleTime){
  //   statusCode = code;
  //   statusCycleTime = cycle;
  //   startCycleMillis = currentMillis;
  // }
}


// Update a single setting value
void updateSetting( uint8_t setting, uint64_t value)
{
  // Map remote setting indexes to receiver settings
  switch( setting ){
    case 0: setting = 0; break;  // TriggerMode
    case 7: setting = 1; break;  // ControlMode
    case 11: setting = 2; break; // Address
  }

  setSettingValue( setting, value);

  updateEEPROMSettings();

  // The address has changed, we need to reinitiate the receiver module
  if(setting == 2) {
    // initRadio(radio);
  }
}

/*
void setCruise ( bool cruise, uint16_t setPoint ){
  if( rxSettings.controlMode == 0 ){

    setThrottle( setPoint );

  }
  else if( rxSettings.controlMode == 1 ){

    setThrottle( setPoint );

  }
  else if( rxSettings.controlMode == 2 ){

    // Setpoint not used (PID by VESC)
    UART.nunchuck.lowerButton = cruise;

    // Make sure the motor doesn't begin to spin wrong way under high load (and don't allow cruise backwards)
    // if( returnData.rpm < 0 ){
    //
    //   UART.nunchuck.lowerButton = false;
    //   UART.nunchuck.valueY = 127;
    //   UART.setNunchuckValues();
    //   UART.setCurrent(0.0);
    //
    // } else{
    //
    //   UART.nunchuck.valueY = 127;
    //   UART.setNunchuckValues();
    //
    // }
  }
} */

void setThrottle(uint16_t value)
{
    throttle = value;

    // UART
    #ifndef FAKE_UART
    UART.nunchuck.valueY = value;
    UART.nunchuck.upperButton = false;
    UART.nunchuck.lowerButton = false;
    UART.setNunchuckValues();
    #endif
    // PPM
    //    digitalWrite(throttlePin, HIGH);
    //    delayMicroseconds(map(throttle, 0, 255, 1000, 2000) );
    //    digitalWrite(throttlePin, LOW);

    // remember throttle for smooth auto stop
    lastThrottle = throttle;
}

void setCruise(uint8_t speed) {

    // UART
    #ifndef FAKE_UART
    UART.nunchuck.valueY = 127;
    UART.nunchuck.upperButton = false;
    UART.nunchuck.lowerButton = true;
    UART.setNunchuckValues();
    #endif
}

// void speedControl( uint16_t throttle , bool trigger )
// {
//   // Kill switch
//   if( rxSettings.triggerMode == 0 ){
//     if ( trigger == true || throttle < 127 ){
//       setThrottle( throttle );
//     }
//     else{
//       setThrottle( default_throttle );
//     }
//   }
//
//   // Cruise control
//   else if( rxSettings.triggerMode == 1 ){
//     if( trigger == true ){
//
//       if( cruising == false ){
//         cruiseThrottle = throttle;
//         cruiseRPM = returnData.rpm;
//         cruising = true;
//       }
//
//       setCruise( true, cruiseThrottle );
//
//     }else{
//       cruising = false;
//       setThrottle( throttle );
//     }
//   }
// }

/*
   Update values used to calculate speed and distance travelled.
*/
void calculateRatios() {
  // Gearing ratio
  gearRatio = (float)boardConfig.motorPulley / (float)boardConfig.wheelPulley;
  // ERPM to Km/h
  ratioRpmSpeed = (gearRatio * 60 * (float)boardConfig.wheelDiameter * 3.14156) / (((float)boardConfig.motorPoles / 2) * 1E6);
  // Pulses to km travelled
  ratioPulseDistance = (gearRatio * (float)boardConfig.wheelDiameter * 3.14156) / (((float)boardConfig.motorPoles * 3) * 1E6);
}

// rpm to km/h
float rpm2speed(long rpm) {
  return ratioRpmSpeed * rpm;
}

// tachometerAbs to km
float tach2dist(long tachometer) {
  return ratioPulseDistance * tachometer;
}

void getUartData()
{
  if (millisSince(lastUartPull) >= uartPullInterval) {

    lastUartPull = millis();

    // debug
    #ifdef FAKE_UART
      telemetry.setVoltage(41.35);
      telemetry.setDistance(rand()%30);
      telemetry.setSpeed(0);
      telemetry.setMotorCurrent(-21);
      telemetry.setInputCurrent(12);
      telemetryUpdated = true;
      delay(7);
      return;
    #endif

    // Only get what we need
    if ( UART.getVescValues() )
    {
      // float dutyCycleNow;
      // float ampHours;
      // float ampHoursCharged;

      telemetry.setVoltage(UART.data.inpVoltage);
      telemetry.setSpeed(rpm2speed(UART.data.rpm));
      telemetry.setDistance(tach2dist(UART.data.tachometerAbs));
      telemetry.setMotorCurrent(UART.data.avgMotorCurrent);
      telemetry.setInputCurrent(UART.data.avgInputCurrent);
      telemetry.tempFET = round(UART.data.tempFET);
      telemetry.tempMotor = round(UART.data.tempMotor);

      lastDelay = millis() - lastUartPull;
      telemetryUpdated = true;

    } else {
      // returnData.ampHours       = 0.0;
      // returnData.inpVoltage     = 0.0;
      // returnData.rpm            = 0;
      // returnData.tachometerAbs  = 0;
      debug("No UART data received!");
    }
  }
}

String uint64ToString(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if(part1 == 0){
    return String(part2, DEC);
  }

  return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}

// Settings functions
void setDefaultEEPROMSettings()
{

  boardID = CPU::getID();
  Serial.println("Board ID: " + String(boardID, HEX));

  boardConfig.maxSpeed = MAX_SPEED; // 30 km/h
  boardConfig.maxRange = MAX_RANGE;      // km
  boardConfig.batteryCells = BATTERY_CELLS;
  boardConfig.batteryType = BATTERY_TYPE;    // 0: Li-ion | 1: LiPo
  boardConfig.motorPoles = MOTOR_POLES;
  boardConfig.wheelDiameter = WHEEL_DIAMETER;
  boardConfig.wheelPulley = WHEEL_PULLEY;
  boardConfig.motorPulley = MOTOR_PULLEY;

  updateEEPROMSettings();
}

void loadEEPROMSettings()
{
  bool rewriteSettings = false;

  // Load settings from EEPROM to custom struct
//  EEPROM.get(0, rxSettings);

  // Loop through all settings to check if everything is fine
  // for ( int i = 0; i < numOfSettings; i++ ) {
  //   int val = getSettingValue(i);
  //
  //   // If setting default value is -1, don't check if its valid
  //   if( settingRules[i][0] != -1 )
  //   {
  //     if ( !inRange( val, settingRules[i][1], settingRules[i][2] ) )
  //     {
  //       // Setting is damaged or never written. Rewrite default.
  //       rewriteSettings = true;
  //       setSettingValue(i, settingRules[i][0] );
  //     }
  //   }

  // }

  // if(rxSettings.firmVersion != VERSION){
  //
  //   setDefaultEEPROMSettings();
  //
  // }
  // else if (rewriteSettings == true)
  // {
  //   updateEEPROMSettings();
  // }
  //
  // debug("Settings loaded");
}

// Write settings to the EEPROM
void updateEEPROMSettings()
{
//  EEPROM.put(0, rxSettings);
}

// Set a value of a specific setting by index.
void setSettingValue(int index, uint64_t value)
{
  // switch (index) {
  //   case 0: rxSettings.triggerMode = value; break;
  //   case 1: rxSettings.controlMode = value; break;
  //   case 2: rxSettings.address = value;     break;
  //
  //   default: /* Do nothing */ break;
  // }
}

// Get settings value by index (usefull when iterating through settings).
int getSettingValue(uint8_t index)
{
  // int value;
  // switch (index) {
  //   case 0: value = rxSettings.triggerMode; break;
  //   case 1: value = rxSettings.controlMode; break;
  //
  //   default: return -1;
  // }
  // return value;
}

bool inRange(int val, int minimum, int maximum)
{
  return ((minimum <= val) && (val <= maximum));
}
