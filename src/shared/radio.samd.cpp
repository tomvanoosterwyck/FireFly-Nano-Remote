#ifdef ARDUINO_SAMD_ZERO

#include "radio.h"
#include "globals.h"
#include <SPI.h>

#include <RH_RF69.h>
// RH_RF69 rdio(RF_CS, RF_DI0);

/*
   Initiate the radio module
*/
void initRadio(RH_RF69 &radio) {

  pinMode(RF_RST, OUTPUT);
  digitalWrite(RF_RST, LOW);

  digitalWrite(RF_RST, HIGH);
  delay(10);
  digitalWrite(RF_RST, LOW);

  delay(10);

  if (!radio.init()) {
    debug("RFM69 radio init failed");
    while (1);
  }

  // Defaults after init are 915.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!radio.setFrequency(RF_FREQ)) {
    debug("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  radio.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the receiver
  // Replace dots with random bytes from this page:
  // https://www.random.org/cgi-bin/randbyte?nbytes=16&format=h
  //  uint8_t key[] = { 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x..,
  //                    0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x.., 0x..};

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };

  radio.setEncryptionKey(key);

  debug(String(RF_FREQ) + " Mhz");

}
#endif
