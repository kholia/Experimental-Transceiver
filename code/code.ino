/*
  Note: Based on https://github.com/pu2clr/SI4735/tree/master/examples/SI47XX_06_ESP8266/OLED_ALL_IN_ONE.

  This sketch runs on ESP8266 device.

  It is  a  complete  radio  capable  to  tune  LW,  MW,  SW  on  AM  and  SSB  mode  and  also  receive  the
  regular  comercial  stations.

  The  purpose  of  this  example  is  to  demonstrate a prototype  receiver based  on  the  SI4735-D60 or Si4732-A10  and  the
  "PU2CLR SI4735 Arduino Library". It is not the purpose of this prototype  to provide you a beautiful interface. You can do it better.


  Features:   AM; SSB; LW/MW/SW; external mute circuit control; AGC; Attenuation gain control;
              SSB filter; CW; AM filter; 1, 5, 10, 50 and 500kHz step on AM and 10Hhz sep on SSB

  ESP8266/ESP12F and components wire up.

  | Device name               | Device Pin / Description      |  ESP8266      |
  | ----------------          | ----------------------------- | ------------  |
  |    OLED                   |                               |               |
  |                           | SDA/SDIO                      |  GPIO4        |
  |                           | SCL/SCLK                      |  GPIO5        |
  |    Encoder                |                               |               |
  |                           | A                             |  GPIO 13      |
  |                           | B                             |  GPIO 14      |
  |                           | PUSH BUTTON (encoder)         |  GPIO 12      |

  ESP8266/ESP12F and SI4735-D60 or SI4732-A10 wire up

  | Si4735  | SI4732   | DESC.  | ESP8266  (GPIO)    |
  |---------| -------- |--------|--------------------|
  | pin 15  |  pin 9   | RESET  |   2 (GPIO2)        |
  | pin 18  |  pin 12  | SDIO   |   4 (SDA / GPIO4)  |
  | pin 17  |  pin 11  | SCLK   |   5 (SCL / GPIO5)  |

  ATTENTION: Read the file user_manual.txt
  Prototype documentation: https://pu2clr.github.io/SI4735/
  PU2CLR Si47XX API documentation: https://pu2clr.github.io/SI4735/extras/apidoc/html/

  By PU2CLR, Ricardo, May  2021.
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "EEPROM.h"
#include <SI4735.h>
#include "DSEG7_Classic_Regular_16.h"
#include "Rotary.h"
#include "patch_init.h" // SSB patch for whole SSBRX initialization string

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//// TX code starts ////

// Si5351 powered digital radio beacon. WeMos D1 Mini.
//
// Author: Dhiru Kholia (VU3CER), 21th-October-2021.
//
// Borrowed from:
// - https://github.com/iw5ejm/multibandWSPR_nodeMCU/blob/master/multibandWSPR_nodeMCU.ino
// - https://randomnerdtutorials.com/esp8266-web-server-spiffs-nodemcu/
// - https://randomnerdtutorials.com/esp8266-relay-module-ac-web-server/
// - https://randomnerdtutorials.com/esp8266-0-96-inch-oled-display-with-arduino-ide/
// - https://www.bakke.online/index.php/2017/06/02/self-updating-ota-firmware-for-esp8266/
//
// Required Libraries
// ------------------
// Etherkit Si5351 (Library Manager)
// Etherkit JTEncode (Library Manager)
// Time (Library Manager)
// Wire (Arduino Standard Library)
// RTClib
//
// https://www.arduinoslovakia.eu/application/timer-calculator

#include <JTEncode.h>
#include <Wire.h>

#include <RTClib.h>
#include <si5351.h>
#include <TimeLib.h>
#include <NTPClient.h>
#include <uEEPROMLib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
// WiFi credentials
#include "credentials.h"

#define ENABLE_OLED 1

// JTEncode jtencode;
// http://wsprnet.org/drupal/node/218 (WSPR Frequencies)
// 40m band -> 7.038600 (dial frequency) 7.040000 - 7.040200 (tx frequency range)
// 30m band -> 10.138700 (dial frequency) 10.140100 - 10.140300 7(tx frequency range)
// 20m band -> 14.095600 (dial frequency) 14.097000 - 14.097200 (tx frequency range)
// unsigned long freq = 7040500UL;              // Change this
// unsigned long freq = 14097200UL;             // Change this
// unsigned long freq = 14097100UL;             // Change this
// unsigned long freq = 10140100UL;             // Change this
// unsigned long freq = 10140000UL;             // This works great for 30m band!
// unsigned long freq = 14097000UL;             // Change this
// unsigned long freq = 14096750UL;             // Change this
// unsigned long freq = 7040500UL;              // Change this
// Note: Generate 'tones' / 'tx_buffer' with WSPR or ft8_lib

// https://www.qsl.net/yo4hfu/SI5351.html says,
//
// If it is necessary, frequency correction must to be applied. My Si5351
// frequency was too high, correction used 1.787KHz at 10MHz. Open again
// Si5351Example sketch, set CLK_0 to 1000000000 (10MHz). Upload. Connect a
// accurate frequency counter to CLK_0 output pin 10. Correction factor:
// (Output frequency Hz - 10000000Hz) x 100. Example: (10001787Hz - 10000000Hz)
// x 100 = 178700 Note: If output frequency is less than 10MHz, use negative
// value of correction, example -178700.

#define CORRECTION 20500 // !!! ATTENTION !!! Change this for your reference oscillator

// https://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
// The built-in led is on pin D4, and it is inverted.

// Global variables
#define PTT_PIN 14 // D5
// #define TX_LED_PIN 12 // D6
#define BUTTON_PIN D7
#define EEPROM_MODE_OFFSET 0
#define EEPROM_DELTA_OFFSET 4
#define EEPROM_TIME_DELTA_HACK_ENABLED_OFFSET 12
#define FREQUENCY_OFFSET 16
#define DELTA_DEFAULT 400

int buttonState = 0;
RTC_DS3231 rtc;
Si5351 si5351;
DateTime dt;
JTEncode jtencode;
uEEPROMLib eeprom(0x57); // I2C EEPROM - incorporated on DS3231 RTC modules
unsigned long freq;
char message[] = "VU3CER VU3FOE MK68";
char call[] = "VU3FOE";
char loc[] = "MK68";
uint8_t dbm = 27;
uint8_t tx_buffer[255];  // NOTE
uint8_t symbol_count;
double tone_delay, tone_spacing;
int rtc_lost_power = 0;
int vfo_ok = 1;
double delta = DELTA_DEFAULT;
uint32_t time_delta_hack_enabled = 0;
int beacon_enabled = 0;
const int ledPin = LED_BUILTIN;
enum codes
{
  NOP,
  LED_FLASH,
  NTP_SYNC,
  TUNE,
  TUNE_LONG,
  TUNE_POWER,
  MODE_SYNC,
  DELTA_SYNC,
  TIME_DELTA_HACK_SYNC,
  FREQUENCY_SYNC
};
enum modes
{
  FT8,
  FT4,
  WSPR,
};
enum modes mode = FT8; // Note
enum codes action = NOP;
#define str(x) #x
#define xstr(x) str(x)
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const long utcOffsetInSeconds = (5.5 * 3600);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

// Loop through the string, transmitting one character at a time.
void tx()
{
  uint8_t i;

  Serial.println("TX!");
#ifdef ENABLE_OLED
  String omode = "Unknown";
  if (mode == FT8)
    omode = String(xstr(FT8));
  else if (mode == FT4)
    omode = String(xstr(FT4));
  else if (mode == WSPR)
    omode = String(xstr(WSPR));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.printf("%lu %s %s\n", freq, omode, "TX!");
  display.display();
#endif

  // Reset the tone to the base frequency and turn on the output
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PTT_PIN, HIGH);// Note

  // Channel jumping hack
  // long n = random(-800, 2300);
  // long n = -759; // Hz, TX around 285 Hz
  // long n = 0;
  // unsigned long nfreq = freq + n;
  unsigned long nfreq = freq;

  for (i = 0; i < symbol_count; i++)
  {
    // si5351.set_freq((freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
    si5351.set_freq((nfreq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
    delay(tone_delay);
  }

  // Turn off the output
  si5351.set_clock_pwr(SI5351_CLK0, 0);
  digitalWrite(PTT_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
#ifdef ENABLE_OLED
  display.clearDisplay();
  display.display();
#endif
}

String getTemperature()
{
  return String(rtc.getTemperature());
}

String getTime()
{
  char date[10] = "hh:mm:ss";
  rtc.now().toString(date);

  return date;
}

// debug helper
void led_flash()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  digitalWrite(LED_BUILTIN, HIGH);
}

// ft8_lib
int ftx_encoder(char *message, uint8_t *tones, int is_ft4);

#define WSPR_DELAY 683 // Delay value for WSPR
#define FT8_DELAY 159  // Delay value for FT8 (12.64 / 79) * 1000 -> 160) // Note: Keeping this to perfect 160 value doesn't work at all!

// From ft8_lib,
// #define FT8_SYMBOL_PERIOD (0.160f) // < FT8 symbol duration, defines tone deviation in Hz and symbol rate
// #define FT8_SLOT_TIME     (15.0f)  // < FT8 slot period
// #define FT4_SYMBOL_PERIOD (0.048f) // < FT4 symbol duration, defines tone deviation in Hz and symbol rate
// #define FT4_SLOT_TIME     (7.5f)   // < FT4 slot period

// From https://github.com/agustinmartino/wsjt_transceiver/blob/master/wsjt_transceiver.ino (for Arduino)
// #define FT8_DELAY               157          // Delay value for FT8
// #define FT4_DELAY                45          // Need to substract 3ms due to PLL delay, value from LU2HES - thanks!
// #define FT4_DELAY                47          // Works for https://github.com/anupamsaikia
#define FT4_DELAY                45 // working a few times!?

#define WSPR_TONE_SPACING 146 // ~1.46 Hz
#define FT8_TONE_SPACING 625  // ~6.25 Hz // (1 / 160.0) * 1000 -> 6.25
// #define FT4_TONE_SPACING 2083.3333 // ~20.83 Hz
#define FT4_TONE_SPACING 2083.3333 // ~20.83 Hz // (1/48.0) * 1000 -> 20.8333

// #define WSPR_DEFAULT_FREQ 14096750UL
#define WSPR_DEFAULT_FREQ 14097050UL  // +300 Hz
#define FT8_DEFAULT_FREQ 14075000UL
#define FT4_DEFAULT_FREQ 14081000UL // +1 KHz

// https://physics.princeton.edu/pulsar/k1jt/FT4_Protocol.pdf
// https://physics.princeton.edu/pulsar/k1jt/FT4_FT8_QEX.pdf
#define FT8_SYMBOL_COUNT 79
#define FT4_SYMBOL_COUNT 74 // working a few times, how is this even possible!?
// #define FT4_SYMBOL_COUNT 105 // not working at all, weird!

void set_mode()
{
  mode = FT8;
  time_delta_hack_enabled = 1;

  if (mode == WSPR) {
    freq = WSPR_DEFAULT_FREQ;
    symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
  } else if (mode == FT8) {
    freq = FT8_DEFAULT_FREQ;
    symbol_count = FT8_SYMBOL_COUNT;
    tone_spacing = FT8_TONE_SPACING;
    tone_delay = FT8_DELAY;
    ftx_encoder(message, tx_buffer, 0);
    Serial.print("FSK tones: ");
    for (int j = 0; j < symbol_count; ++j) {
      Serial.print(tx_buffer[j]);
    }
    Serial.println("\n");
  } else if (mode == FT4) {
    freq = FT4_DEFAULT_FREQ;
    symbol_count = FT4_SYMBOL_COUNT;
    tone_spacing = FT4_TONE_SPACING;
    tone_delay = FT4_DELAY;
    ftx_encoder(message, tx_buffer, 1);
    Serial.print("FSK tones: ");
    for (int j = 0; j < symbol_count; ++j) {
      Serial.print(tx_buffer[j]);
    }
    Serial.println("\n");
  }
}

//// TX code ends ////


const uint16_t size_content = sizeof ssb_patch_content; // see patch_init.h

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define RESET_PIN 2                // GPIO2

// Enconder PINs
#define ENCODER_PIN_A 13           // GPIO13
#define ENCODER_PIN_B 14           // GPIO14


// Buttons controllers
#define ENCODER_PUSH_BUTTON 12     // GPIO12

#define MIN_ELAPSED_TIME 300
#define MIN_ELAPSED_RSSI_TIME 200
#define ELAPSED_COMMAND 2000  // time to turn off the last command controlled by encoder. Time to goes back to the FVO control
#define ELAPSED_CLICK 1500    // time to check the double click commands
#define DEFAULT_VOLUME 35    // change it for your favorite sound volume

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4

#define SSB 1

#define EEPROM_SIZE        512

#define STORE_TIME 10000 // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).

// EEPROM - Stroring control variables
const uint8_t app_id = 47; // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
long storeTime = millis();

bool itIsTimeToSave = false;

bool bfoOn = false;
bool ssbLoaded = false;

int8_t agcIdx = 0;
uint8_t disableAgc = 0;
int8_t agcNdx = 0;
int8_t softMuteMaxAttIdx = 4;
uint8_t countClick = 0;

uint8_t seekDirection = 1;

bool cmdBand = false;
bool cmdVolume = false;
bool cmdAgc = false;
bool cmdBandwidth = false;
bool cmdStep = false;
bool cmdMode = false;
bool cmdMenu = false;
bool cmdSoftMuteMaxAtt = false;

bool fmRDS = false;

int16_t currentBFO = 0;
long elapsedRSSI = millis();
long elapsedButton = millis();

long elapsedClick = millis();
long elapsedCommand = millis();
volatile int encoderCount = 0;
uint16_t currentFrequency;

const uint8_t currentBFOStep = 10;

const char *menu[] = {"Volume", "Step", "Mode", "BFO", "BW", "AGC/Att", "SoftMute", "Seek Up", "Seek Down"};
int8_t menuIdx = 0;
const int lastMenu = 8;
int8_t currentMenuCmd = -1;


typedef struct
{
  uint8_t idx;      // SI473X device bandwidth index
  const char *desc; // bandwidth description
} Bandwidth;

int8_t bwIdxSSB = 4;
const int8_t maxSsbBw = 5;
Bandwidth bandwidthSSB[] = {
  {4, "0.5"},
  {5, "1.0"},
  {0, "1.2"},
  {1, "2.2"},
  {2, "3.0"},
  {3, "4.0"}
};


int8_t bwIdxAM = 4;
const int8_t maxAmBw = 6;
Bandwidth bandwidthAM[] = {
  {4, "1.0"},
  {5, "1.8"},
  {3, "2.0"},
  {6, "2.5"},
  {2, "3.0"},
  {1, "4.0"},
  {0, "6.0"}
};

int8_t bwIdxFM = 0;
const int8_t maxFmBw = 4;

Bandwidth bandwidthFM[] = {
  {0, "AUT"}, // Automatic - default
  {1, "110"}, // Force wide (110 kHz) channel filter.
  {2, " 84"},
  {3, " 60"},
  {4, " 40"}
};



int tabAmStep[] = {1,    // 0
                   5,    // 1
                   9,    // 2
                   10,   // 3
                   50,   // 4
                   100
                  }; // 5

const int lastAmStep = (sizeof tabAmStep / sizeof(int)) - 1;
int idxAmStep = 3;

int tabFmStep[] = {5, 10, 20};
const int lastFmStep = (sizeof tabFmStep / sizeof(int)) - 1;
int idxFmStep = 1;

uint16_t currentStepIdx = 1;


const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};
uint8_t currentMode = FM;

/**
    Band data structure
*/
typedef struct
{
  const char *bandName;   // Band description
  uint8_t bandType;       // Band type (FM, MW or SW)
  uint16_t minimumFreq;   // Minimum frequency of the band
  uint16_t maximumFreq;   // maximum frequency of the band
  uint16_t currentFreq;   // Default frequency or current frequency
  int8_t currentStepIdx;  // Idex of tabStepAM:  Defeult frequency step (See tabStepAM)
  int8_t bandwidthIdx;    // Index of the table bandwidthFM, bandwidthAM or bandwidthSSB;
} Band;

/*
   Band table
   YOU CAN CONFIGURE YOUR OWN BAND PLAN. Be guided by the comments.
   To add a new band, all you have to do is insert a new line in the table below. No extra code will be needed.
   You can remove a band by deleting a line if you do not want a given band.
   Also, you can change the parameters of the band.
   ATTENTION: You have to RESET the eeprom after adding or removing a line of this table.
              Turn your receiver on with the encoder push button pressed at first time to RESET the eeprom content.
*/
Band band[] = {
  {"VHF", FM_BAND_TYPE, 6400, 10800, 10390, 1, 0},
  {"MW1", MW_BAND_TYPE, 150, 1720, 810, 3, 4},
  {"MW2", MW_BAND_TYPE, 531, 1701, 783, 2, 4},
  {"MW2", MW_BAND_TYPE, 1700, 3500, 2500, 1, 4},
  {"80M", MW_BAND_TYPE, 3500, 4000, 3700, 0, 4},
  {"SW1", SW_BAND_TYPE, 4000, 5500, 4885, 1, 4},
  {"SW2", SW_BAND_TYPE, 5500, 6500, 6000, 1, 4},
  {"40M", SW_BAND_TYPE, 6500, 7300, 7100, 0, 4},
  {"SW3", SW_BAND_TYPE, 7200, 8000, 7200, 1, 4},
  {"SW4", SW_BAND_TYPE, 9000, 11000, 9500, 1, 4},
  {"SW5", SW_BAND_TYPE, 11100, 13000, 11900, 1, 4},
  {"SW6", SW_BAND_TYPE, 13000, 14000, 13500, 1, 4},
  {"20M", SW_BAND_TYPE, 14000, 15000, 14200, 0, 4},
  {"SW7", SW_BAND_TYPE, 15000, 17000, 15300, 1, 4},
  {"SW8", SW_BAND_TYPE, 17000, 18000, 17500, 1, 4},
  {"15M", SW_BAND_TYPE, 20000, 21400, 21100, 0, 4},
  {"SW9", SW_BAND_TYPE, 21400, 22800, 21500, 1, 4},
  {"CB ", SW_BAND_TYPE, 26000, 28000, 27500, 0, 4},
  {"10M", SW_BAND_TYPE, 28000, 30000, 28400, 0, 4},
  {"ALL", SW_BAND_TYPE, 150, 30000, 15000, 0, 4} // All band. LW, MW and SW (from 150kHz to 30MHz)
};

const int lastBand = (sizeof band / sizeof(Band)) - 1;
int bandIdx = 0;
int tabStep[] = {1, 5, 10, 50, 100, 500, 1000};
const int lastStep = (sizeof tabStep / sizeof(int)) - 1;


uint8_t rssi = 0;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

SI4735 rx;


void setup()
{
  // Encoder pins
  pinMode(ENCODER_PUSH_BUTTON, INPUT_PULLUP);

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  //// TX code starts ///
  int ret = 0;
  char date[10] = "hh:mm:ss";
  // Safety first!
  pinMode(PTT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(PTT_PIN, LOW);
  // Setup serial and IO pins
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n...");
  // Initialize the Si5351
  ret = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, CORRECTION);
  if (ret != true) {
    Serial.flush();
    vfo_ok = 0;
  }
  si5351.set_clock_pwr(SI5351_CLK0, 0); // safety first
  Serial.print("Si5351 init status (should be 1 always) = ");
  Serial.println(ret);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    Serial.flush();
    abort();
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, abort!?");
    Serial.flush();
    rtc_lost_power = 1;
  }
  rtc.disable32K();
  rtc.now().toString(date);
  Serial.print("Current time is = ");
  Serial.println(date);
  Serial.print("Temperature is: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
  si5351.set_freq(freq * 100, SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for maximum power
  si5351.set_clock_pwr(SI5351_CLK0, 0); // Disable the clock initially
  set_mode();
  if (mode == WSPR) {
    freq = WSPR_DEFAULT_FREQ;
  } else if (mode == FT8) {
    freq = FT8_DEFAULT_FREQ;
  } else if (mode == FT4) {
    freq = FT4_DEFAULT_FREQ;
  }
  if (!vfo_ok) {
    Serial.println("Check VFO connections!");
    led_flash();
    delay(50);
  }
  if (rtc_lost_power) {
    Serial.println("Check and set RTC time!");
    led_flash();
    delay(50);
  }
  if (rtc_lost_power || !vfo_ok) {
    beacon_enabled = 0;
  } else {
    beacon_enabled = 1;
    led_flash();
  }
  //// TX code ends ///

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  display.display();
  display.setTextColor(SSD1306_WHITE);

  // Splash - Remove or change it for your introduction text.
  display.clearDisplay();
  print(0, 0, NULL, 2, "PU2CLR");
  print(0, 15, NULL, 2, "ESP8266");
  display.display();
  delay(500);
  display.clearDisplay();
  print(0, 0, NULL, 2, "SI473X");
  print(0, 15, NULL, 2, "Arduino");
  display.display();
  // End Splash
  delay(500);
  display.clearDisplay();

  EEPROM.begin(EEPROM_SIZE);

  // If you want to reset the eeprom, keep the VOLUME_UP button pressed during statup
  if (digitalRead(ENCODER_PUSH_BUTTON) == LOW)
  {
    EEPROM.write(eeprom_address, 0);
    EEPROM.commit();
    print(0, 0, NULL, 2, "EEPROM RESETED");
    delay(3000);
    display.clearDisplay();
  }

#ifdef HAVE_UNOBTANIUM_CHIP
  // ICACHE_RAM_ATTR void rotaryEncoder(); see rotaryEncoder implementation below.
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

  rx.setI2CFastModeCustom(100000);

  rx.getDeviceI2CAddress(RESET_PIN); // Looks for the I2C bus address and set it.  Returns 0 if error

  rx.setup(RESET_PIN, MW_BAND_TYPE);
  // Comment the line above and uncomment the three lines below if you are using external ref clock (active crystal or signal generator)
  // rx.setRefClock(32768);
  // rx.setRefClockPrescaler(1);   // will work with 32768
  // rx.setup(RESET_PIN, 0, MW_BAND_TYPE, SI473X_ANALOG_AUDIO, XOSCEN_RCLK);

  delay(300);

  // Checking the EEPROM content
  if (EEPROM.read(eeprom_address) == app_id)
  {
    readAllReceiverInformation();
  } else
    rx.setVolume(volume);

  useBand();
  showStatus();
#endif
}

/**
   Prints a given content on display
*/
void print(uint8_t col, uint8_t lin, const GFXfont *font, uint8_t textSize, const char *msg) {
  display.setFont(font);
  display.setTextSize(textSize);
  display.setCursor(col, lin);
  display.print(msg);
}

void printParam(const char *msg) {
  display.fillRect(0, 10, 128, 10, SSD1306_BLACK);
  print(0, 10, NULL, 1, msg);
  display.display();
}

/*
   writes the conrrent receiver information into the eeprom.
   The EEPROM.update avoid write the same data in the same memory position. It will save unnecessary recording.
*/
void saveAllReceiverInformation()
{
  int addr_offset;

  EEPROM.begin(EEPROM_SIZE);

  EEPROM.write(eeprom_address, app_id);                 // stores the app id;
  EEPROM.write(eeprom_address + 1, rx.getVolume()); // stores the current Volume
  EEPROM.write(eeprom_address + 2, bandIdx);            // Stores the current band
  EEPROM.write(eeprom_address + 3, fmRDS);
  EEPROM.write(eeprom_address + 4, currentMode); // Stores the current Mode (FM / AM / SSB)
  EEPROM.write(eeprom_address + 5, currentBFO >> 8);
  EEPROM.write(eeprom_address + 6, currentBFO & 0XFF);
  EEPROM.commit();

  addr_offset = 7;
  band[bandIdx].currentFreq = currentFrequency;

  for (int i = 0; i <= lastBand; i++)
  {
    EEPROM.write(addr_offset++, (band[i].currentFreq >> 8));   // stores the current Frequency HIGH byte for the band
    EEPROM.write(addr_offset++, (band[i].currentFreq & 0xFF)); // stores the current Frequency LOW byte for the band
    EEPROM.write(addr_offset++, band[i].currentStepIdx);       // Stores current step of the band
    EEPROM.write(addr_offset++, band[i].bandwidthIdx);         // table index (direct position) of bandwidth
    EEPROM.commit();
  }

  EEPROM.end();
}

/**
   reads the last receiver status from eeprom.
*/
void readAllReceiverInformation()
{
  uint8_t volume;
  int addr_offset;
  int bwIdx;
  EEPROM.begin(EEPROM_SIZE);

  volume = EEPROM.read(eeprom_address + 1); // Gets the stored volume;
  bandIdx = EEPROM.read(eeprom_address + 2);
  fmRDS = EEPROM.read(eeprom_address + 3);
  currentMode = EEPROM.read(eeprom_address + 4);
  currentBFO = EEPROM.read(eeprom_address + 5) << 8;
  currentBFO |= EEPROM.read(eeprom_address + 6);

  addr_offset = 7;
  for (int i = 0; i <= lastBand; i++)
  {
    band[i].currentFreq = EEPROM.read(addr_offset++) << 8;
    band[i].currentFreq |= EEPROM.read(addr_offset++);
    band[i].currentStepIdx = EEPROM.read(addr_offset++);
    band[i].bandwidthIdx = EEPROM.read(addr_offset++);
  }

  EEPROM.end();

  currentFrequency = band[bandIdx].currentFreq;

  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    currentStepIdx = idxFmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);
  }
  else
  {
    currentStepIdx = idxAmStep = band[bandIdx].currentStepIdx;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
  }

  bwIdx = band[bandIdx].bandwidthIdx;

  if (currentMode == LSB || currentMode == USB)
  {
    loadSSB();
    bwIdxSSB = (bwIdx > 5) ? 5 : bwIdx;
    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSBBSidebandCutoffFilter(0);
    else
      rx.setSBBSidebandCutoffFilter(1);
  }
  else if (currentMode == AM)
  {
    bwIdxAM = bwIdx;
    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  }
  else
  {
    bwIdxFM = bwIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }

  delay(50);
  rx.setVolume(volume);
}

/*
   To store any change into the EEPROM, it is needed at least STORE_TIME  milliseconds of inactivity.
*/
void resetEepromDelay()
{

  elapsedCommand = storeTime = millis();
  itIsTimeToSave = true;
}

/**
    Set all command flags to false
    When all flags are disabled (false), the encoder controls the frequency
*/
void disableCommands()
{
  cmdBand = false;
  bfoOn = false;
  cmdVolume = false;
  cmdAgc = false;
  cmdBandwidth = false;
  cmdStep = false;
  cmdMode = false;
  cmdMenu = false;
  cmdSoftMuteMaxAtt = false;
  countClick = 0;
  // showCommandStatus((char *) "VFO ");
}

/**
   Reads encoder via interrupt
   Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
   if you do not add ICACHE_RAM_ATTR declaration, the system will reboot during attachInterrupt call.
   With ICACHE_RAM_ATTR macro you put the function on the RAM.
*/
ICACHE_RAM_ATTR void  rotaryEncoder()
{ // rotary encoder events
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus)
    encoderCount = (encoderStatus == DIR_CW) ? 1 : -1;
}

/**
   Shows frequency information on Display
*/
void showFrequency()
{
  char tmp[15];
  char bufferDisplay[15];
  char * unit;
  sprintf(tmp, "%5.5u", currentFrequency);
  bufferDisplay[0] = (tmp[0] == '0') ? ' ' : tmp[0];
  bufferDisplay[1] = tmp[1];
  if (rx.isCurrentTuneFM())
  {
    bufferDisplay[2] = tmp[2];
    bufferDisplay[3] = '.';
    bufferDisplay[4] = tmp[3];
    unit = (char *) "MHz";
  }
  else
  {
    if ( currentFrequency  < 1000 ) {
      bufferDisplay[1] = ' ';
      bufferDisplay[2] = tmp[2] ;
      bufferDisplay[3] = tmp[3];
      bufferDisplay[4] = tmp[4];
    } else {
      bufferDisplay[2] = tmp[2];
      bufferDisplay[3] = tmp[3];
      bufferDisplay[4] = tmp[4];
    }
    unit = (char *) "kHz";
  }
  bufferDisplay[5] = '\0';
  // strcat(bufferDisplay, unit);
  // display.setTextSize(2);
  display.setFont(&DSEG7_Classic_Regular_16);
  display.clearDisplay();
  display.setCursor(20, 24);
  display.print(bufferDisplay);
  display.setCursor(90, 15);
  display.setFont(NULL);
  display.setTextSize(1);
  display.print(unit);
  display.display();

  showMode();
}

/**
   Shows the current mode
*/
void showMode() {
  char * bandMode;
  if (currentFrequency < 520)
    bandMode = (char *) "LW  ";
  else
    bandMode = (char *) bandModeDesc[currentMode];

  display.setTextSize(1);
  // display.clearDisplay();
  display.setCursor(0, 0);
  display.print(bandMode);
  display.setCursor(90, 0);
  display.print(band[bandIdx].bandName);
  display.display();
}

/**
   Shows some basic information on display
*/
void showStatus()
{
  // lcd.clear();
  showFrequency();
  showRSSI();
}

/**
    Shows the current Bandwidth status
*/
void showBandwidth()
{
  char *bw;
  char bandwidth[20];
  if (currentMode == LSB || currentMode == USB)
  {
    bw = (char *)bandwidthSSB[bwIdxSSB].desc;
    showBFO();
  }
  else if (currentMode == AM)
  {
    bw = (char *)bandwidthAM[bwIdxAM].desc;
  }
  else
  {
    bw = (char *)bandwidthFM[bwIdxFM].desc;
  }
  sprintf(bandwidth, "BW: %s", bw);
  printParam(bandwidth);
}

/**
     Shows the current RSSI and SNR status
*/
void showRSSI()
{
  char sMeter[10];
  sprintf(sMeter, "S:%d ", rssi);

  display.fillRect(0, 25, 128, 10, SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(80, 25);
  display.print(sMeter);
  if (currentMode == FM)
  {
    display.setCursor(0, 25);
    display.print((rx.getCurrentPilot()) ? "ST" : "MO");
  }

  display.display();
}

/**
      Shows the current AGC and Attenuation status
*/
void showAgcAtt()
{
  char sAgc[15];
  // lcd.clear();
  rx.getAutomaticGainControl();
  if (agcNdx == 0 && agcIdx == 0)
    strcpy(sAgc, "AGC ON");
  else
    sprintf(sAgc, "ATT: %2.2d", agcNdx);

  printParam(sAgc);
}

/**
     Shows the current step
*/
void showStep()
{
  char sStep[15];
  sprintf(sStep, "Stp:%4d", (currentMode == FM) ? (tabFmStep[currentStepIdx] * 10) : tabAmStep[currentStepIdx]);
  printParam(sStep);
}

/**
    Shows the current BFO value
*/
void showBFO()
{
  char bfo[18];

  if (currentBFO > 0)
    sprintf(bfo, "BFO: +%4.4d", currentBFO);
  else
    sprintf(bfo, "BFO: %4.4d", currentBFO);

  printParam(bfo);
  elapsedCommand = millis();
}

/*
    Shows the volume level on LCD
*/
void showVolume()
{
  char volAux[12];
  sprintf(volAux, "VOLUME: %2u", rx.getVolume());
  printParam(volAux);
}

/**
   Show Soft Mute
*/
void showSoftMute()
{
  char sMute[18];
  sprintf(sMute, "Soft Mute: %2d", softMuteMaxAttIdx);
  printParam(sMute);
}

void setBand(int8_t up_down)
{
  band[bandIdx].currentFreq = currentFrequency;
  band[bandIdx].currentStepIdx = currentStepIdx;
  if (up_down == 1)
    bandIdx = (bandIdx < lastBand) ? (bandIdx + 1) : 0;
  else
    bandIdx = (bandIdx > 0) ? (bandIdx - 1) : lastBand;
  useBand();
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
  elapsedCommand = millis();
}

/**
   Switch the radio to current band
*/
void useBand()
{
  if (band[bandIdx].bandType == FM_BAND_TYPE)
  {
    currentMode = FM;
    rx.setTuneFrequencyAntennaCapacitor(0);
    rx.setFM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabFmStep[band[bandIdx].currentStepIdx]);
    rx.setSeekFmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq);
    bfoOn = ssbLoaded = false;
    bwIdxFM = band[bandIdx].bandwidthIdx;
    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }
  else
  {
    // set the tuning capacitor for SW or MW/LW
    rx.setTuneFrequencyAntennaCapacitor((band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) ? 0 : 1);
    if (ssbLoaded)
    {
      rx.setSSB(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabAmStep[band[bandIdx].currentStepIdx], currentMode);
      rx.setSSBAutomaticVolumeControl(1);
      rx.setSsbSoftMuteMaxAttenuation(softMuteMaxAttIdx); // Disable Soft Mute for SSB
      bwIdxSSB = band[bandIdx].bandwidthIdx;
      rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    }
    else
    {
      currentMode = AM;
      rx.setAM(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq, band[bandIdx].currentFreq, tabAmStep[band[bandIdx].currentStepIdx]);
      bfoOn = false;
      bwIdxAM = band[bandIdx].bandwidthIdx;
      rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
      rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx); // Soft Mute for AM or SSB
    }
    rx.setAutomaticGainControl(disableAgc, agcNdx);
    rx.setSeekAmLimits(band[bandIdx].minimumFreq, band[bandIdx].maximumFreq); // Consider the range all defined current band
    rx.setSeekAmSpacing(5); // Max 10kHz for spacing

  }
  delay(100);
  currentFrequency = band[bandIdx].currentFreq;
  currentStepIdx = band[bandIdx].currentStepIdx;

  rssi = 0;
  showStatus();
  showCommandStatus((char *) "Band");
}


void loadSSB() {
  rx.setI2CFastModeCustom(400000); // You can try rx.setI2CFastModeCustom(700000); or greater value
  rx.loadPatch(ssb_patch_content, size_content, bandwidthSSB[bwIdxSSB].idx);
  rx.setI2CFastModeCustom(100000);
  ssbLoaded = true;
}

/**
    Switches the Bandwidth
*/
void doBandwidth(int8_t v)
{
  if (currentMode == LSB || currentMode == USB)
  {
    bwIdxSSB = (v == 1) ? bwIdxSSB + 1 : bwIdxSSB - 1;

    if (bwIdxSSB > maxSsbBw)
      bwIdxSSB = 0;
    else if (bwIdxSSB < 0)
      bwIdxSSB = maxSsbBw;

    rx.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
    if (bandwidthSSB[bwIdxSSB].idx == 0 || bandwidthSSB[bwIdxSSB].idx == 4 || bandwidthSSB[bwIdxSSB].idx == 5)
      rx.setSBBSidebandCutoffFilter(0);
    else
      rx.setSBBSidebandCutoffFilter(1);

    band[bandIdx].bandwidthIdx = bwIdxSSB;
  }
  else if (currentMode == AM)
  {
    bwIdxAM = (v == 1) ? bwIdxAM + 1 : bwIdxAM - 1;

    if (bwIdxAM > maxAmBw)
      bwIdxAM = 0;
    else if (bwIdxAM < 0)
      bwIdxAM = maxAmBw;

    rx.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
    band[bandIdx].bandwidthIdx = bwIdxAM;

  } else {
    bwIdxFM = (v == 1) ? bwIdxFM + 1 : bwIdxFM - 1;
    if (bwIdxFM > maxFmBw)
      bwIdxFM = 0;
    else if (bwIdxFM < 0)
      bwIdxFM = maxFmBw;

    rx.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    band[bandIdx].bandwidthIdx = bwIdxFM;
  }
  showBandwidth();
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

/**
   Show cmd on display. It means you are setting up something.
*/
void showCommandStatus(char * currentCmd)
{
  display.fillRect(40, 0, 50, 8, SSD1306_BLACK);
  display.setCursor(40, 0);
  display.print(currentCmd);
  display.display();
}

/**
   Show menu options
*/
void showMenu() {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.print(menu[menuIdx]);
  display.display();
  showCommandStatus( (char *) "Menu");
}

/**
    AGC and attenuattion setup
*/
void doAgc(int8_t v) {
  agcIdx = (v == 1) ? agcIdx + 1 : agcIdx - 1;
  if (agcIdx < 0 )
    agcIdx = 35;
  else if ( agcIdx > 35)
    agcIdx = 0;
  disableAgc = (agcIdx > 0); // if true, disable AGC; esle, AGC is enable
  if (agcIdx > 1)
    agcNdx = agcIdx - 1;
  else
    agcNdx = 0;
  rx.setAutomaticGainControl(disableAgc, agcNdx); // if agcNdx = 0, no attenuation
  showAgcAtt();
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
  elapsedCommand = millis();
}


/**
   Switches the current step
*/
void doStep(int8_t v)
{
  if ( currentMode == FM ) {
    idxFmStep = (v == 1) ? idxFmStep + 1 : idxFmStep - 1;
    if (idxFmStep > lastFmStep)
      idxFmStep = 0;
    else if (idxFmStep < 0)
      idxFmStep = lastFmStep;

    currentStepIdx = idxFmStep;
    rx.setFrequencyStep(tabFmStep[currentStepIdx]);

  } else {
    idxAmStep = (v == 1) ? idxAmStep + 1 : idxAmStep - 1;
    if (idxAmStep > lastAmStep)
      idxAmStep = 0;
    else if (idxAmStep < 0)
      idxAmStep = lastAmStep;

    currentStepIdx = idxAmStep;
    rx.setFrequencyStep(tabAmStep[currentStepIdx]);
    rx.setSeekAmSpacing(5); // Max 10kHz for spacing
  }
  band[bandIdx].currentStepIdx = currentStepIdx;
  showStep();
  elapsedCommand = millis();
}

/**
   Switches to the AM, LSB or USB modes
*/
void doMode(int8_t v)
{
  if (currentMode != FM)
  {
    if (v == 1)  { // clockwise
      if (currentMode == AM)
      {
        // If you were in AM mode, it is necessary to load SSB patch (avery time)
        loadSSB();
        ssbLoaded = true;
        currentMode = LSB;
      }
      else if (currentMode == LSB)
        currentMode = USB;
      else if (currentMode == USB)
      {
        currentMode = AM;
        bfoOn = ssbLoaded = false;
      }
    } else { // and counterclockwise
      if (currentMode == AM)
      {
        // If you were in AM mode, it is necessary to load SSB patch (avery time)
        loadSSB();
        ssbLoaded = true;
        currentMode = USB;
      }
      else if (currentMode == USB)
        currentMode = LSB;
      else if (currentMode == LSB)
      {
        currentMode = AM;
        bfoOn = ssbLoaded = false;
      }
    }
    // Nothing to do if you are in FM mode
    band[bandIdx].currentFreq = currentFrequency;
    band[bandIdx].currentStepIdx = currentStepIdx;
    useBand();
  }
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
  elapsedCommand = millis();
}

/**
   Sets the audio volume
*/
void doVolume( int8_t v ) {
  if ( v == 1)
    rx.volumeUp();
  else
    rx.volumeDown();

  showVolume();
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
}

/**
    This function is called by the seek function process.
*/
void showFrequencySeek(uint16_t freq)
{
  currentFrequency = freq;
  showFrequency();
}

/**
    Find a station. The direction is based on the last encoder move clockwise or counterclockwise
*/
void doSeek()
{
  if ((currentMode == LSB || currentMode == USB)) return; // It does not work for SSB mode

  rx.seekStationProgress(showFrequencySeek, seekDirection);
  currentFrequency = rx.getFrequency();

}

/**
   Sets the Soft Mute Parameter
*/
void doSoftMute(int8_t v)
{
  softMuteMaxAttIdx = (v == 1) ? softMuteMaxAttIdx + 1 : softMuteMaxAttIdx - 1;
  if (softMuteMaxAttIdx > 32)
    softMuteMaxAttIdx = 0;
  else if (softMuteMaxAttIdx < 0)
    softMuteMaxAttIdx = 32;

  rx.setAmSoftMuteMaxAttenuation(softMuteMaxAttIdx);
  showSoftMute();
  elapsedCommand = millis();
}

/**
    Menu options selection
*/
void doMenu( int8_t v) {
  menuIdx = (v == 1) ? menuIdx + 1 : menuIdx - 1;
  if (menuIdx > lastMenu)
    menuIdx = 0;
  else if (menuIdx < 0)
    menuIdx = lastMenu;

  showMenu();
  delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
  elapsedCommand = millis();
}


/**
   Starts the MENU action process
*/
void doCurrentMenuCmd() {
  disableCommands();
  switch (currentMenuCmd) {
    case 0:                 // VOLUME
      cmdVolume = true;
      showVolume();
      break;
    case 1:                 // STEP
      cmdStep = true;
      showStep();
      break;
    case 2:                 // MODE
      cmdMode = true;
      showMode();
      break;
    case 3:
      bfoOn = true;
      if ((currentMode == LSB || currentMode == USB)) {
        showBFO();
      }
      // showFrequency();
      break;
    case 4:                 // BW
      cmdBandwidth = true;
      showBandwidth();
      break;
    case 5:                 // AGC/ATT
      cmdAgc = true;
      showAgcAtt();
      break;
    case 6:
      cmdSoftMuteMaxAtt = true;
      showSoftMute();
      break;
    case 7:
      seekDirection = 1;
      doSeek();
      break;
    case 8:
      seekDirection = 0;
      doSeek();
      break;
    default:
      showStatus();
      break;
  }
  currentMenuCmd = -1;
  elapsedCommand = millis();
}

/**
   Return true if the current status is Menu command
*/
bool isMenuMode() {
  return (cmdMenu | cmdStep | cmdBandwidth | cmdAgc | cmdVolume | cmdSoftMuteMaxAtt | cmdMode);
}

/**
   Main loop
*/
void loop()
{
  // Check if the encoder has moved.
  if (encoderCount != 0)
  {
    if (bfoOn & (currentMode == LSB || currentMode == USB))
    {
      currentBFO = (encoderCount == 1) ? (currentBFO + currentBFOStep) : (currentBFO - currentBFOStep);
      rx.setSSBBfo(currentBFO);
      showBFO();
    }
    else if (cmdMenu)
      doMenu(encoderCount);
    else if (cmdMode)
      doMode(encoderCount);
    else if (cmdStep)
      doStep(encoderCount);
    else if (cmdAgc)
      doAgc(encoderCount);
    else if (cmdBandwidth)
      doBandwidth(encoderCount);
    else if (cmdVolume)
      doVolume(encoderCount);
    else if (cmdSoftMuteMaxAtt)
      doSoftMute(encoderCount);
    else if (cmdBand)
      setBand(encoderCount);
    else
    {
      if (encoderCount == 1)
      {
        rx.frequencyUp();
      }
      else
      {
        rx.frequencyDown();
      }
      // Show the current frequency only if it has changed
      currentFrequency = rx.getFrequency();
      showFrequency();
    }
    encoderCount = 0;
    resetEepromDelay();
  }
  else
  {
    if (digitalRead(ENCODER_PUSH_BUTTON) == LOW)
    {
      countClick++;
      if (cmdMenu)
      {
        currentMenuCmd = menuIdx;
        doCurrentMenuCmd();
      }
      else if (countClick == 1)
      { // If just one click, you can select the band by rotating the encoder
        if (isMenuMode())
        {
          disableCommands();
          showStatus();
          showCommandStatus((char *)"VFO ");
        }
        else if (bfoOn) {
          bfoOn = false;
          showStatus();
        }
        else
        {
          cmdBand = !cmdBand;
          showCommandStatus((char *)"Band");
        }
      }
      else
      { // GO to MENU if more than one click in less than 1/2 seconds.
        cmdMenu = !cmdMenu;
        if (cmdMenu)
          showMenu();
      }
      delay(MIN_ELAPSED_TIME);
      elapsedCommand = millis();
    }
  }

  // Show RSSI status only if this condition has changed
  if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 6)
  {
    rx.getCurrentReceivedSignalQuality();
    int aux = rx.getCurrentRSSI();
    if (rssi != aux && !isMenuMode())
    {
      rssi = aux;
      showRSSI();
    }
    elapsedRSSI = millis();
  }

  // Disable commands control
  if ((millis() - elapsedCommand) > ELAPSED_COMMAND)
  {
    if ((currentMode == LSB || currentMode == USB) )
    {
      bfoOn = false;
      // showBFO();
      showStatus();
    } else if (isMenuMode())
      showStatus();
    disableCommands();
    elapsedCommand = millis();
  }

  if ((millis() - elapsedClick) > ELAPSED_CLICK)
  {
    countClick = 0;
    elapsedClick = millis();
  }

  // Show the current frequency only if it has changed
  if (itIsTimeToSave)
  {
    if ((millis() - storeTime) > STORE_TIME)
    {
      saveAllReceiverInformation();
      storeTime = millis();
      itIsTimeToSave = false;
    }
  }

  // CAT control here
  if (Serial.available() > 0)
  {
    char key = Serial.read();
    switch (key)
    {
      case '+':
        rx.volumeUp();
        break;
      case '-':
        rx.volumeDown();
        break;
      case 'a':
      case 'A':
        rx.setAM(570, 1710, 810, 10);
        break;
      case 'f':
      case 'F':
        rx.setFM(8600, 10800, 10390, 10);
        break;
      case '1':
        rx.setAM(9400, 9990, 9600, 5);
        break;
      case 'U':
      case 'u':
        rx.frequencyUp();
        break;
      case 'D':
      case 'd':
        rx.frequencyDown();
        break;
      case 'b':
      case 'B':
        //
        break;
      case 'S':
        rx.seekStationUp();
        break;
      case 's':
        rx.seekStationDown();
        break;
      case '0':
        //
        break;
      case '?':
        //
        break;
      case 'v':
        Serial.println("VVV");
        break;
      case 'T':
        if (beacon_enabled) {
          while (1)
          {
            dt = rtc.now();
            if (time_delta_hack_enabled == 1 && (dt.second() % 15 == 14) && mode == FT8) { // Check for 14th, 29th, 44th, 59th second
              // if (time_delta_hack_enabled == 1 && (dt.second() == 14) && mode == FT8) { // Check for 14th second only!
              delay(delta);
              tx();
              break;
            }
            else if (dt.second() == 0 && (dt.minute() % 2) == 0 && mode == WSPR) {
              tx();
              break;
            }
            else
              delay(50);
          }
        }
        break;
      default:
        break;
    }
  }

  delay(5);
}
