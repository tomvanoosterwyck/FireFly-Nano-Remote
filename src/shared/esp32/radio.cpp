#ifdef ESP32

#include "radio.h"
#include "globals.h"
#include <SPI.h>

#include <LoRa.h>

/*
   Initiate the radio module
*/
void initRadio() {

  // reset radio
  pinMode(RF_RST, OUTPUT);

  digitalWrite(RF_RST, LOW);
  delay(10);
  digitalWrite(RF_RST, HIGH);

  SPI.begin (RF_SCK, RF_MISO, RF_MOSI, RF_CS);
  LoRa.setPins (RF_CS, RF_RST, RF_DI0);

  LoRa.setSPIFrequency(10E6);

  if (! LoRa.begin (RF_FREQ * 1E6)) {
    debug ("Starting LoRa failed!");
    while (1);
  }

  debug("LoRa Init OK!");

  LoRa.setSignalBandwidth(500E3);
  // set tx power
  LoRa.setTxPower(14);

  LoRa.setSpreadingFactor(6);

  // crc on
  LoRa.enableCrc();

}

#endif
