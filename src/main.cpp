// Copyright 2021 Marc-Antoine Ruel. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

#include <ArduinoOTA.h>
#include <Button2.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <esp_adc_cal.h>
#include <LeptonFLiR.h>


// Assert that WIFI_SSID was provided.
static_assert(sizeof(WIFI_SSID) > 2, "Don't forget to source secrets.sh");


// Standard ESP32 SPI pins:
//      HSPI VSPI
// CS0*   15    5
// SCLK   14   18
// MISO   12   19
// MOSI   13   23
// https://github.com/espressif/arduino-esp32/blob/master/libraries/SPI/src/SPI.cpp

// LilyGO TTGO-Display pin out
//
// GND        3V3
// GND         36
// 21 SDA      37
// 22 SCL      38
// 17          39
//  2          32
// 15          33 CS
// 13          25 SCLK
// 12          26
// GND         27 MISO
// GND        GND
// 3V3         5V


// Globals.


// ADC_EN is the ADC detection enable port:
// - If the USB port is used for power supply, it is turned on by default.
// - If it is powered by battery, it needs to be set to HIGH. After voltage
//   measurement, it needs to be set to LOW to save power.
const int ADC_EN = 14;
// ADC_PIN is the pin to read to do voltage measurement.
const int ADC_PIN = 34;
// GPIO for physical buttons.
const int BUTTON_LEFT = 0;
const int BUTTON_RIGHT = 35;


// API documentation is code:
//   https://github.com/Bodmer/TFT_eSPI/blob/master/TFT_eSPI.h
//
// TTGO-Display:
// - 135x240; default view is in portrait, which is what we want.
// - Top left is 0, 0.
// - Bottom right is 134, 239.
// - RGB 65k colors
//
// Pins (VSPI) as defined in platform.ini:
// - SCLK    19
// - CS       5
// - MOSI    19
// - MISO    23 (unused)
// - TFT_BL   4 (backlight, HIGH = on) Only for v1.1, v1.0 used 14.
// - TFT_DC  16 (???)
// - TFT_RST 23 (unnecessary)
#ifdef USE_DISPLAY
TFT_eSPI tft;
#endif


// https://github.com/LennartHennigs/Button2
Button2 button_left(BUTTON_LEFT);
Button2 button_right(BUTTON_RIGHT);


// Lepton IR camera over SPI.
const int IR_SPI_CS = 33;
const int IR_SPI_SCLK = 25;
const int IR_SPI_MISO = 27;
const int IR_SPI_MOSI = 26; // unused, but we have to sacrifice one
const int I2C_SDA = 21;
const int I2C_SCL = 22;
LeptonFLiR flir(Wire, IR_SPI_CS);

// To skip redundant frames.
uint32_t flirLastFrameNumber = -1;


// Voltage reference for USB or LiPo voltage measurement.
int vref = 1100;


int lastOTAPercent = 0;

// For next 1 second timeout.
uint32_t nextSecondMillis = 0;
uint32_t nextDeciSecondMillis = 0;
//TaskHandle_t loopDisplayHandle = NULL;
#ifdef MARUEL_TIMERS
TimerHandle_t timerDisplay = NULL;
TimerHandle_t timerLepton = NULL;
#endif


// Display

// Code for the clock, to be removed.
// Saved H, M, S x & y coords.
uint16_t osx=64, osy=64, omx=64, omy=64, ohx=64, ohy=64;

static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

// Get H, M, S from compile time.
uint8_t hh = conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);

boolean initialClockUpdate = 1;

enum BOTTOM_DISPLAY {
  BOTTOM_NOTHING = 0,
  BOTTOM_VOLTAGE = 1,
  BOTTOM_RSSI = 2,
  BOTTOM_IP = 3,
  BOTTOM_DISPLAY_END = 4,
} bottomDisplay = BOTTOM_NOTHING;


void displayBottom(const char *msg, int color) {
#ifdef USE_DISPLAY
  const int height = 18;
  const int32_t Y_BOTTOM = (TFT_HEIGHT-height);
  tft.fillRect(0, Y_BOTTOM, TFT_WIDTH, height, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(msg, 0, Y_BOTTOM, 2);
#endif
}

void displayBottom(const String& msg, int color) {
  displayBottom(msg.c_str(), color);
}

void displayBottom2(const char *msg, int color) {
#ifdef USE_DISPLAY
  const int height = 18;
  const int32_t Y_BOTTOM = (TFT_HEIGHT-(2*height));
  tft.fillRect(0, Y_BOTTOM, TFT_WIDTH, height, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(msg, 0, Y_BOTTOM, 2);
#endif
  Serial.print("bottom:");
  Serial.println(msg);
}

void displayBottom2(const String& msg, int color) {
  displayBottom2(msg.c_str(), color);
}


// Clock code.

// drawClock draws the background clock.
void drawClock() {
#ifdef USE_DISPLAY
  int TFT_GREY = 0xBDF7;
  tft.fillScreen(TFT_GREY);
  tft.setTextColor(TFT_GREEN, TFT_GREY);

  // Draw clock face.
  tft.fillCircle(64, 64, 61, TFT_BLUE);
  tft.fillCircle(64, 64, 57, TFT_BLACK);

  // Draw 12 lines.
  for (int i = 0; i<360; i+= 30) {
    float sx = cos((i-90)*0.0174532925);
    float sy = sin((i-90)*0.0174532925);
    uint16_t x0 = sx*57+64;
    uint16_t yy0 = sy*57+64;
    uint16_t x1 = sx*50+64;
    uint16_t yy1 = sy*50+64;

    tft.drawLine(x0, yy0, x1, yy1, TFT_BLUE);
  }

  // Draw 60 dots.
  for (int i = 0; i<360; i+= 6) {
    float sx = cos((i-90)*0.0174532925);
    float sy = sin((i-90)*0.0174532925);
    uint16_t x0 = sx*53+64;
    uint16_t yy0 = sy*53+64;

    tft.drawPixel(x0, yy0, TFT_BLUE);
    if (i==0 || i==180) {
      tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    }
    if (i==0 || i==180) {
      tft.fillCircle(x0+1, yy0, 1, TFT_CYAN);
    }
    if (i==90 || i==270) {
      tft.fillCircle(x0, yy0, 1, TFT_CYAN);
    }
    if (i==90 || i==270) {
      tft.fillCircle(x0+1, yy0, 1, TFT_CYAN);
    }
  }

  tft.fillCircle(65, 65, 3, TFT_RED);

  // Draw text at position 64,125 using fonts 4
  // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . a p m
  // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
  tft.drawCentreString("Time flies", TFT_WIDTH/2, 130, 4);
  tft.drawString("v1", 0, 160, 4);

  // Test corners:
  tft.fillRect(0, 0, 5, 5, TFT_BLUE);
  tft.fillRect(0, 235, 5, 5, TFT_GREEN);
  tft.fillRect(130, 0, 5, 5, TFT_ORANGE);
  tft.fillRect(130, 235, 5, 5, TFT_MAGENTA);
#endif
}

void updateClock() {
#ifdef USE_DISPLAY
  // Pre-compute hand degrees, x & y coords for a fast screen update.
  float sdeg = ss*6;                  // 0-59 -> 0-354
  float mdeg = mm*6+sdeg*0.01666667;  // 0-59 -> 0-360 - includes seconds
  float hdeg = hh*30+mdeg*0.0833333;  // 0-11 -> 0-360 - includes minutes and seconds
  float hx = cos((hdeg-90)*0.0174532925);
  float hy = sin((hdeg-90)*0.0174532925);
  float mx = cos((mdeg-90)*0.0174532925);
  float my = sin((mdeg-90)*0.0174532925);
  float sx = cos((sdeg-90)*0.0174532925);
  float sy = sin((sdeg-90)*0.0174532925);

  if (ss==0 || initialClockUpdate) {
    initialClockUpdate = 0;
    // Erase hour and minute hand positions every minute.
    tft.drawLine(ohx, ohy, 65, 65, TFT_BLACK);
    ohx = hx*33+65;
    ohy = hy*33+65;
    tft.drawLine(omx, omy, 65, 65, TFT_BLACK);
    omx = mx*44+65;
    omy = my*44+65;
  }

  // Redraw new hand positions, hour and minute hands not erased here to avoid
  // flicker.
  tft.drawLine(osx, osy, 65, 65, TFT_BLACK);
  tft.drawLine(ohx, ohy, 65, 65, TFT_WHITE);
  tft.drawLine(omx, omy, 65, 65, TFT_WHITE);
  osx = sx*47+65;
  osy = sy*47+65;
  tft.drawLine(osx, osy, 65, 65, TFT_RED);

  tft.fillCircle(65, 65, 3, TFT_RED);
#endif
}

void initVoltage() {
  //adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_0db);
  //adc1_config_width(ADC_WIDTH_12Bit);
  //adc1_get_voltage(ADC1_CHANNEL_6);
  // For LiPo voltage, do:
  // pinMode(ADC_EN, OUTPUT);
  // Before measuring voltage: digitalWrite(ADC_EN, HIGH);
  // To preserve power: digitalWrite(ADC_EN, LOW);

  // Check type of calibration value used to characterize ADC.
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    vref = adc_chars.vref;
  }
  pinMode(ADC_PIN, INPUT);
  pinMode(ADC_EN, OUTPUT);
}

void showVoltage() {
  //float battery_voltage = (analogRead(ADC_PIN) / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  int battery_voltage = (analogRead(ADC_PIN) * 66 * vref + 40950/2) / 40950;
  String msg(battery_voltage);
  msg += "mV";
  displayBottom(msg, TFT_GREEN);
}

void showRSSI() {
  long rssi = WiFi.RSSI();
  String msg("RSSI: ");
  msg += String(rssi);
  displayBottom(msg, TFT_GREEN);
}

void showIP() {
  displayBottom(WiFi.localIP().toString(), TFT_GREEN);
}

// Housekeeping code.

// fromchar converts a hex char into its base 16 value.
char fromchar(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return c - 'a' + 10;
}

// fromhex converts an hex encoded string inplace back to ASCII. Used to be able
// to load passwords with characters that are tricky to pass from shell
// environment variable to C++ define to C++ strings here.
void fromhex(char *s) {
  int l = strlen(s);
  for (int i = 0; i < l/2; i++) {
    s[i] = (fromchar(s[2*i]) << 4) + fromchar(s[2*i+1]);
  }
  s[l/2] = 0;
}

// display turns the display backlight on or off.
void display(bool on) {
  digitalWrite(TFT_BL, on ? HIGH : LOW);
}

// deepSleep turns off the display and sets the left button as the wake up
// signal.
void deepSleep() {
  display(false);
#ifdef USE_DISPLAY
  tft.writecommand(TFT_DISPOFF);
  tft.writecommand(TFT_SLPIN);
#endif
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_deep_sleep_start();
}

// shallowSleep is for long delays without turning fully off.
void shallowSleep(int ms) {
	esp_sleep_enable_timer_wakeup(ms * 1000);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	esp_light_sleep_start();
}


// handleDisplay runs at 1Hz.
void handleDisplay(void*) {
  Serial.printf("%u handleDisplay()\n", (uint32_t)millis());
  // Advance second.
  ss++;
  if (ss==60) {
    ss=0;
    // Advance minute.
    mm++;
    if (mm>59) {
      mm=0;
      // Advance hour.
      hh++;
      if (hh>23) {
        hh=0;
      }
    }
  }
  updateClock();
  switch (bottomDisplay) {
    case BOTTOM_NOTHING:
      displayBottom("PressLeft", TFT_GREEN);
      break;
    case BOTTOM_DISPLAY_END:
      displayBottom("", TFT_GREEN);
      break;
    case BOTTOM_VOLTAGE:
      showVoltage();
      break;
    case BOTTOM_RSSI:
      showRSSI();
      break;
    case BOTTOM_IP:
      showIP();
      break;
  }
  Serial.printf("%u handleDisplay() end\n", (uint32_t)millis());
}

// handleLepton runs at 10Hz.
void handleLepton(void*) {
  Serial.printf("%u handleLepton()\n", (uint32_t)millis());
  if (flir.readNextFrame()) {
    uint32_t frameNumber = flir.getTelemetryFrameCounter();
    if (frameNumber != flirLastFrameNumber) {
      flirLastFrameNumber = frameNumber;
      // Find the hottest spot on the frame.
      int hotVal = 0;
      int hotX = 0;
      int hotY = 0;
      for (int y = 0; y < flir.getImageHeight(); ++y) {
        for (int x = 0; x < flir.getImageWidth(); ++x) {
          int val = flir.getImageDataRowCol(y, x);
          if (val > hotVal) {
            hotVal = val;
            hotX = x; hotY = y;
          }
        }
      }

      // TODO(maruel): This should be done by the display task.
      String msg(hotVal);
      msg += " (";
      msg += hotX;
      msg += ",";
      msg += hotY;
      displayBottom2(msg, TFT_WHITE);

      // TODO(maruel): Display frame.
      //if (flir.getShouldRunFFCNormalization()) {
      //  flir.sys_runFFCNormalization();
      //}
    }
  }
  /*
  //String msg(flir.sys_getCameraUptime());
  String msg(flir.sys_getAuxTemperature());
  msg += "`C";
  displayBottom2(msg, TFT_WHITE);
  */
  Serial.printf("%u handleLepton() end\n", (uint32_t)millis());
}

// Initialization.
void setup() {
  // The CP2104 works fine at high speed.
  // Don't forget to update monitor_speed in ../platform.ini.
  Serial.begin(1843200);

  // Wifi.
  // TODO(maruel): Neither hostname nor events work.
  WiFi.setHostname("ircam-esp32");
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    String msg;
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      msg = "IP:";
      msg += WiFi.localIP();
      displayBottom(msg, TFT_GREEN);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      displayBottom("Disconnected", TFT_GREEN);
      // We could try to auto reconnect but it's probably easier for the user to
      // reset the device, so do not do anything for now.
      break;
    default:
      break;
    }
  });
  // hex decode so we can pass weird characters through environment variables.
  char pass[] = WIFI_PASS;
  fromhex(pass);
  WiFi.begin(WIFI_SSID, (const char *)pass);

  // Camera.
  // Change to 400000 if not stable..
  Wire.begin(I2C_SDA, I2C_SCL, 1000000);
  // An ESP32 specific implementation is used. Documentation is code:
  //   https://github.com/espressif/arduino-esp32/blob/master/libraries/SPI/src/SPI.h
  // The lepton code uses its own CS line handling so we must not configure the
  // SPI one properly (!) with IR_SPI_CS. Instead sacrifice GPIO 32 for this, it's
  // not currently used. The CS line for the lepton never seems to be raised by
  // the driver so I think the implementation is incorrect.
  SPI.begin(IR_SPI_SCLK, IR_SPI_MISO, IR_SPI_MOSI, 32);
  flir.init(LeptonFLiR_ImageStorageMode_80x60_16bpp);
  flir.sys_setTelemetryEnabled(ENABLED);

  // Buttons.
  button_left.setPressedHandler([](Button2 & b) {
    bottomDisplay = (BOTTOM_DISPLAY)((bottomDisplay+1)%BOTTOM_DISPLAY_END);
  });

  button_right.setPressedHandler([](Button2 & b) {
  });
  initVoltage();

  // Display and Clock.
#ifdef USE_DISPLAY
  tft.init();
#endif
  drawClock();
  displayBottom("Welcome!", TFT_WHITE);
  displayBottom2("FLIR", TFT_WHITE);

  nextDeciSecondMillis = millis() + 100;
  nextSecondMillis = nextDeciSecondMillis + 900;

  // OTA
  ArduinoOTA
    .onStart([]() {
#ifdef MARUEL_TIMERS
      xTimerStop(timerDisplay, 100/portTICK_PERIOD_MS);
      xTimerStop(timerLepton, 100/portTICK_PERIOD_MS);
#endif
      // The display will continue updating asynchronously.
      bottomDisplay = BOTTOM_NOTHING;
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      // TODO(maruel): Not use SPIFFS anyway.
      // In practice the following will not have time to be shown. It's only in
      // case it somehow hangs after.
      displayBottom("OTA" + type, TFT_YELLOW);
    })
    .onEnd([]() {
      // In practice the following will not have time to be shown. It's only in
      // case it somehow hangs after.
      displayBottom("Updated", TFT_YELLOW);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // It's important to throttle draws otherwise it severely slows down the
      // OTA process.
      // Intentionally do not round up:
      int percent = (100 * progress) / total;
      if (progress == 0 || lastOTAPercent != percent) {
        lastOTAPercent = percent;
        displayBottom(String("OTA: " + String(percent) + "%"), TFT_YELLOW);
      }
    })
    .onError([](ota_error_t error) {
      const char * msg;
      switch (error) {
        case OTA_AUTH_ERROR:
          msg = "Auth Failed";
          break;
        case OTA_BEGIN_ERROR:
          msg = "Begin Failed";
          break;
        case OTA_CONNECT_ERROR:
          msg = "Connect Failed";
          break;
        case OTA_RECEIVE_ERROR:
          msg = "Receive Failed";
          break;
        case OTA_END_ERROR:
          msg = "End Failed";
          break;
        default:
          msg = "Unknown Error";
      }
      displayBottom(msg, TFT_YELLOW);
    });
  ArduinoOTA.begin();

  // loop() will be called in a loop on the application core. Create a second
  // task to loop on the protocol core to maximize utilization.
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/freertos-smp.html
  // PRO_CPU = 0 (Protocols CPU, Bluetooth and Wifi)
  // APP_CPU = 1 (Application CPU)
  //int core2 = xPortGetCoreID() ^ 1;
  // Run the display on the protocol core since the display is less demanding
  // that the lepton, which reads the HSPI bus continuously.
  //xTaskCreatePinnedToCore(loopDisplay, "loop2", CONFIG_ARDUINO_LOOP_STACK_SIZE, NULL, 10, &loop2Handle, PRO_CPU);

#ifdef MARUEL_TIMERS
  // TODO(maruel): Investigate using timers. The doc is unclear if using float
  // is fine in these, as for task it notes that tasks will be pinned to a CPU.
  timerDisplay = xTimerCreate("display", 1000/portTICK_PERIOD_MS, pdTRUE, NULL, handleDisplay);
  timerLepton = xTimerCreate("lepton", 100/portTICK_PERIOD_MS, pdTRUE, NULL, handleLepton);
  xTimerStart(timerDisplay, 100/portTICK_PERIOD_MS);
  xTimerStart(timerLepton, 100/portTICK_PERIOD_MS);
#endif
}


// It's called from
// https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/main.cpp
void loop() {
  // Once a second operations.
  // TODO(maruel): 49 days.
  uint32_t m = millis();
  if (nextSecondMillis <= m) {
    nextSecondMillis = m+1000;
    handleDisplay(NULL);
  }

  if (nextDeciSecondMillis <= m) {
    nextDeciSecondMillis = m + 100;
    handleLepton(NULL);
  }

  ArduinoOTA.handle();

  button_left.loop();
  button_right.loop();

  // Force a task scheduling and throttle ourselves to 50Hz~100Hz max.
  //vTaskDelay(10 / portTICK_PERIOD_MS);
  delay(1);
}
