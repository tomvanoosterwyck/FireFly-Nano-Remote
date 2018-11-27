#include <stdint.h>

/*  Hardware related definitions for TTGO V2.1 Board
// ATTENTION: check your board version!
// This settings are for boards labeled v1.6 on pcb, NOT for v1.5 or older
*/

// Pin definitions
#define LED           25   // LED on board
#define PIN_BUTTON    14   // RX - pushbutton pin
#define PIN_TRIGGER   RX
#define PIN_THROTTLE  34
#define PIN_BATTERY   35

// #define PIN_VIBRO     ?

#define HAS_DISPLAY  // OLED-Display on board
#define DISPLAY_ROTATION  3

#define HAS_BATTERY_PROBE ADC1_GPIO35_CHANNEL // uses GPIO7
#define BATT_FACTOR 2 // voltage divider 100k/100k on board

// Pins for I2C interface of OLED Display
#define DISPLAY_SDA 21
#define DISPLAY_SCL 22
#define DISPLAY_RST 16

// Pins for LORA chip SPI interface, reset line and interrupt lines
#define RF_SCK  5
#define RF_CS   18
#define RF_MISO 19
#define RF_MOSI 27
#define RF_RST  23
#define RF_DI0  26
#define RF_DI1  33
#define RF_DI2  32
