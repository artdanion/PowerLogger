/*--------------------------------------------------------------------------------
Powerlogger Project based on:
ESP32 3-Channel Power Logger Project
https://hackaday.io/project/187504-esp32-3-channel-power-logger

ported to a Platformio project
added WiFiManager

This Project uses the ESP32C3 ... see platformio.ini

The MIT License (MIT)
Copyright (c) 2023 artdanion, Ovidiu
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
--------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <Wire.h>
#include <ina3221.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "SPIFFS.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ESP32Time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "powerlogger_bmp.h"

#ifndef STASSID
#define STASSID "WIFI"
#define STAPSK "PASSWORD"
#endif

// ----- Define Pins ----- //
#define I2C_SDA 10
#define I2C_SCL 8
#define SCK 3
#define MISO 4
#define MOSI 5
#define SDCARD_CS 6
#define TFT_CS 2
#define TFT_RST 20
#define TFT_DC 21
#define TFT_BL 1
#define LEFT_BUTTON_PIN 7
#define RIGHT_BUTTON_PIN 9
// select which pin will trigger the configuration portal when set to LOW
#define TRIGGER_PIN 7
// ----- Define Pins ----- //

// ----- Define Some Colors ----- //
#define ST7735_BLACK 0x0000
#define ST7735_RED 0x001F
#define ST7735_GREEN 0x07E0
#define ST7735_WHITE 0xFFFF
#define ST7735_BLUE 0xF800
#define ST7735_DEEP_BLUE 0x3800
#define ST7735_YELLOW 0x07FF
#define ST7735_CYAN 0xFFE0
#define ST7735_MAGENTA 0xF81F
// ----- Define Some Colors ----- //

// ----- Initialize TFT ----- //
#define ST7735_TFTWIDTH 128
#define ST7735_TFTHEIGHT 160
#define background_color ST7735_DEEP_BLUE
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// ----- Initialize TFT ----- //

// ----- Define ina3221 Address ----- //
INA3221 ina3221(INA3221_ADDR40_GND); // Solder J1 Pad
#define CHANNEL1 INA3221_CH1
#define CHANNEL2 INA3221_CH2
#define CHANNEL3 INA3221_CH3
// ----- Define ina3221 Address ----- //

ESP32Time rtc;
void ICACHE_RAM_ATTR handle_left_Interrupt();
void ICACHE_RAM_ATTR handle_right_Interrupt();

// ------- Initialize WiFiManager ------- //
WiFiManager wifiManager;
unsigned int timeout = 120; // seconds to run for

// ------- Initialize WiFiManager ------- //

void boot_sequesnce();
void setup_menu();
void measure_values();
void displaydata();
void write_file();
void wakeDisplay();
void sleepDisplay();
void convert_time();
float get_battery_voltage();
void create_file();
void extractIpAddress(char *sourceString, short *ipAddress);

// ----- Define Variables ----- //
const char *ssid = STASSID;
const char *password = STAPSK;

int backlight_pwm = 255; // Start with the display brightness at 100%
int left_button_flag = 0;
int right_button_flag = 0;
int selected = 1;
int selected_avg = 1;
int channel_number = 1; // The default channel to display at startup

static unsigned long last_interrupt_time = 0; // Used in order to debounce the buttons
unsigned long interrupt_time = 0;             // Used in order to debounce the buttons

bool started = false;
bool display_state = true; // Display is awake when true and sleeping when false
bool ignore_input = false; // Used in order to ingnore the buttons

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 200; // Update data on screen and on SD Card every 200ms
unsigned long display_on_time = 0;
unsigned long start_delay = 0;

float shunt_voltage_1 = 0;
float bus_voltage_1 = 0;
float current_mA_1 = 0;
float load_voltage_1 = 0;
float energy_1 = 0;
float capacity_1 = 0;

float shunt_voltage_2 = 0;
float bus_voltage_2 = 0;
float current_mA_2 = 0;
float load_voltage_2 = 0;
float energy_2 = 0;
float capacity_2 = 0;

float shunt_voltage_3 = 0;
float bus_voltage_3 = 0;
float current_mA_3 = 0;
float load_voltage_3 = 0;
float energy_3 = 0;
float capacity_3 = 0;

float battery_voltage = 0;

bool use_sd_card = true;
bool use_channel_1 = true;
bool use_channel_2 = true;
bool use_channel_3 = true;
bool setup_error = false;
bool file_active = false;

String file_name = "/log.txt"; // Default file name in case the is an error with the NTP server

// ----- Time variables ----- //
const long utcOffsetInSeconds = 7200;
unsigned long rtcOffset = 0;
unsigned long seconds = 0;
unsigned long minutes = 0;
unsigned long hours = 0;
unsigned long days = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
// ----- Time variables ----- //

void setup()
{
  Serial.begin(115200);
  Serial.println("");

  pinMode(TFT_BL, OUTPUT);
  analogWrite(TFT_BL, backlight_pwm);
  pinMode(LEFT_BUTTON_PIN, INPUT);
  pinMode(RIGHT_BUTTON_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  SPI.begin(SCK, MISO, MOSI, SDCARD_CS);

  ina3221.begin();
  ina3221.reset();
  ina3221.setShuntRes(10, 10, 10);                  // You must specify the shunt resistor values for calibration (in mOhm)
  ina3221.setFilterRes(10, 10, 10);                 // You must specify the filter resistor values for calibration (in Ohm)
  ina3221.setAveragingMode(INA3221_REG_CONF_AVG_1); // The INA module supports internal averaging which is better than using a smooting capacitor

  // ----- Initiate the TFT display ----- //
  tft.initR();
  tft.setCursor(0, 0);
  tft.setRotation(3);
  tft.fillScreen(background_color);

  // for (int y = 0; y < 128; y++)
  // {
  //   for (int x = 0; x < 160; x++)
  //   {
  //     tft.drawPixel(x, y, bitmap[(y * 160) + x]);
  //   }
  // }
  
  tft.drawRGBBitmap(0,0,powerLogger_bmp,160,128);
  // ----- Initiate the TFT display ----- //

  delay(2000);

  // ----- Enable Buttons ----- //
  attachInterrupt(digitalPinToInterrupt(LEFT_BUTTON_PIN), handle_left_Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_BUTTON_PIN), handle_right_Interrupt, FALLING);
  // ----- Enable Buttons ----- //

  // ----- Run the startup checks ----- //
  Serial.println("starting bootsequence...");
  boot_sequesnce();
  // ----- Run the startup checks ----- //

  delay(2000);

  // ----- Run the setup menu ----- //
  setup_menu();
  // ----- Run the setup menu ----- //

  tft.fillScreen(background_color);

  display_on_time = millis();
  start_delay = millis();
}

void loop()
{
  currentMillis = millis();
  analogWrite(TFT_BL, backlight_pwm); // Each loop adjust the brightness of the display

  // ----- Handle measuring and writing data ----- //
  if (currentMillis - previousMillis >= interval)
  {
    if (started)
    {
      measure_values();
      previousMillis = currentMillis;
      displaydata();
      if (use_sd_card == true)
      {
        write_file();
      }
    }
  }
  // ----- Handle measuring and writing data ----- //

  // ----- Handle left button being pressed ----- //
  if (left_button_flag == 1 && display_state == true)
  {
    if (started == true)
    {
      channel_number = channel_number + 1;
      if (channel_number == 1 && use_channel_1 == false)
      {
        channel_number = channel_number + 1;
      }
      if (channel_number == 2 && use_channel_2 == false)
      {
        channel_number = channel_number + 1;
      }
      if (channel_number == 3 && use_channel_3 == false)
      {
        channel_number = channel_number + 1;
      }
      if (channel_number > 3)
      {
        if (use_channel_1 == false)
        {
          if (use_channel_2 == false)
          {
            channel_number = 3;
          }
          else
          {
            channel_number = 2;
          }
        }
        else
        {
          channel_number = 1;
        }
      }
    }

    left_button_flag = 0;
  }
  // ----- Handle left button being pressed ----- //

  // ----- Handle right button being pressed ----- //
  if (right_button_flag == 1 && display_state == true)
  {
    started = false;
    selected = 1;
    right_button_flag = 0;
    shunt_voltage_1 = 0;
    bus_voltage_1 = 0;
    current_mA_1 = 0;
    load_voltage_1 = 0;
    energy_1 = 0;
    capacity_1 = 0;
    shunt_voltage_2 = 0;
    bus_voltage_2 = 0;
    current_mA_2 = 0;
    load_voltage_2 = 0;
    energy_2 = 0;
    capacity_2 = 0;
    shunt_voltage_3 = 0;
    bus_voltage_3 = 0;
    current_mA_3 = 0;
    load_voltage_3 = 0;
    energy_3 = 0;
    capacity_3 = 0;
    file_active = false;
    setup_menu();
  }
  // ----- Handle right button being pressed ----- //

  // ----- Wake the display (increase brightness) ----- //
  if (right_button_flag == 1 && display_state == false)
  {
    wakeDisplay();
    right_button_flag = 0;
  }
  if (left_button_flag == 1 && display_state == false)
  {
    wakeDisplay();
    left_button_flag = 0;
  }
  // ----- Wake the display (increase brightness) ----- //

  // ----- Sleep the display (decrease brightness) ----- //
  if (millis() - display_on_time > 30000)
  {
    sleepDisplay();
  }
  // ----- Sleep the display (decrease brightness) ----- //
}

void displaydata()
{
  String current_padding = "";
  String power_padding = "";
  String energy_padding = "";
  String capacity_padding = "";
  float current_mA = 0;
  float load_voltage = 0;
  float capacity = 0;
  float energy = 0;
  float bus_voltage = 0;

  // ----- Display data of the selected channel ----- //
  if (channel_number == 1)
  {
    bus_voltage = bus_voltage_1;
    current_mA = current_mA_1;
    load_voltage = load_voltage_1;
    energy = energy_1;
    capacity = capacity_1;
  }
  else if (channel_number == 2)
  {
    bus_voltage = bus_voltage_2;
    current_mA = current_mA_2;
    load_voltage = load_voltage_2;
    energy = energy_2;
    capacity = capacity_2;
  }
  else if (channel_number == 3)
  {
    bus_voltage = bus_voltage_3;
    current_mA = current_mA_3;
    load_voltage = load_voltage_3;
    energy = energy_3;
    capacity = capacity_3;
  }
  // ----- Display data of the selected channel ----- //

  // ----- Some padding so that text is properly aligned ----- //
  if (current_mA >= 0 && current_mA < 10)
  {
    current_padding = "   ";
  }
  else if (current_mA >= 10 && current_mA < 100)
  {
    current_padding = "  ";
  }
  else if (current_mA >= 100 && current_mA < 1000)
  {
    current_padding = " ";
  }
  else if (current_mA >= 1000 && current_mA < 10000)
  {
    current_padding = "";
  }

  if (load_voltage * current_mA >= 0 && load_voltage * current_mA < 10)
  {
    power_padding = "   ";
  }
  else if (load_voltage * current_mA >= 10 && load_voltage * current_mA < 100)
  {
    power_padding = "  ";
  }
  else if (load_voltage * current_mA >= 100 && load_voltage * current_mA < 1000)
  {
    power_padding = " ";
  }
  else if (load_voltage * current_mA >= 1000 && load_voltage * current_mA < 10000)
  {
    power_padding = "";
  }

  if (energy >= 0 && energy < 10)
  {
    energy_padding = "   ";
  }
  else if (energy >= 10 && energy < 100)
  {
    energy_padding = "  ";
  }
  else if (energy >= 100 && energy < 1000)
  {
    energy_padding = " ";
  }
  else if (energy >= 1000 && energy < 10000)
  {
    energy_padding = "";
  }

  if (capacity >= 0 && capacity < 10)
  {
    capacity_padding = "   ";
  }
  else if (capacity >= 10 && capacity < 100)
  {
    capacity_padding = "  ";
  }
  else if (capacity >= 100 && capacity < 1000)
  {
    capacity_padding = " ";
  }
  else if (capacity >= 1000 && capacity < 10000)
  {
    capacity_padding = "";
  }
  // ----- Some padding so that text is properly aligned ----- //

  // ----- Display the data ----- //
  tft.setTextSize(2);
  tft.setTextColor(ST7735_YELLOW, background_color);
  tft.setCursor(0, 0);

  convert_time();
  tft.print("T: ");
  tft.print(days);
  tft.print(":");
  if (hours < 10)
  {
    tft.print("0");
  }
  tft.print(hours);
  tft.print(":");
  if (minutes < 10)
  {
    tft.print("0");
  }
  tft.print(minutes);
  tft.print(":");
  if (seconds < 10)
  {
    tft.print("0");
  }
  tft.println(seconds);

  tft.println("          ");

  tft.setTextColor(ST7735_WHITE, background_color);
  tft.print("V:       ");
  tft.println(load_voltage);
  tft.print("mA:   ");
  tft.print(current_padding);
  tft.println(current_mA, 2);
  tft.print("mW:   ");
  tft.print(power_padding);
  tft.println(load_voltage * current_mA, 2);
  tft.print("mWh:  ");
  tft.print(energy_padding);
  tft.println(energy, 2);
  tft.print("mAh:  ");
  tft.print(capacity_padding);
  tft.println(capacity, 2);
  tft.setTextColor(ST7735_RED, background_color);
  tft.print("CH:");
  tft.print(channel_number);
  tft.print("   B:");
  tft.print(get_battery_voltage());
  // ----- Display the data ----- //
}

void convert_time()
{
  unsigned long elapsedMillis = currentMillis - start_delay;
  seconds = elapsedMillis / 1000;
  minutes = seconds / 60;
  hours = minutes / 60;
  days = hours / 24;
  elapsedMillis %= 1000;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;
}

void sleepDisplay()
{
  backlight_pwm = 5;
  display_state = false;
}

void wakeDisplay()
{
  backlight_pwm = 255;
  display_state = true;
}

void measure_values()
{
  if (use_channel_1)
  {
    shunt_voltage_1 = ina3221.getShuntVoltage(INA3221_CH1); // Value will be returned in uV.
    bus_voltage_1 = ina3221.getVoltage(INA3221_CH1);
    current_mA_1 = ina3221.getCurrent(INA3221_CH1) * 100; // Value will be returned in Amps. Adjust if you need mA
    load_voltage_1 = bus_voltage_1 + (shunt_voltage_1 / 1000000);
    energy_1 = energy_1 + (load_voltage_1 * current_mA_1 * interval) / 3600000;
    capacity_1 = capacity_1 + (current_mA_1 * interval) / 3600000;
  }
  if (use_channel_2)
  {
    shunt_voltage_2 = ina3221.getShuntVoltage(INA3221_CH2); // Value will be returned in uV.
    bus_voltage_2 = ina3221.getVoltage(INA3221_CH2);
    current_mA_2 = ina3221.getCurrent(INA3221_CH2) * 100; // Value will be returned in Amps. Adjust if you need mA
    load_voltage_2 = bus_voltage_2 + (shunt_voltage_2 / 1000000);
    energy_2 = energy_2 + (load_voltage_2 * current_mA_2 * interval) / 3600000;
    capacity_2 = capacity_2 + (current_mA_2 * interval) / 3600000;
  }
  if (use_channel_3)
  {
    shunt_voltage_3 = ina3221.getShuntVoltage(INA3221_CH3); // Value will be returned in uV.
    bus_voltage_3 = ina3221.getVoltage(INA3221_CH3);
    current_mA_3 = ina3221.getCurrent(INA3221_CH3) * 100; // Value will be returned in Amps. Adjust if you need mA
    load_voltage_3 = bus_voltage_3 + (shunt_voltage_3 / 1000000);
    energy_3 = energy_3 + (load_voltage_3 * current_mA_3 * interval) / 3600000;
    capacity_3 = capacity_3 + (current_mA_3 * interval) / 3600000;
  }
}

void boot_sequesnce()
{
  WiFi.mode(WIFI_STA);

  // wifiManager.resetSettings(); reset ESP WiFi Settings for test

  wifiManager.setEnableConfigPortal(false);
  wifiManager.setConfigPortalBlocking(false);

  wifiManager.autoConnect();

  tft.fillScreen(background_color);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_RED, background_color);
  tft.setTextWrap(false);

  tft.println("Booting ...");
  tft.println("");
  tft.setTextColor(ST7735_WHITE, background_color);

  // ----- Check if SD Card is OK ----- //
  tft.print("SD Card");
  if (!SD.begin(SDCARD_CS))
  {
    Serial.println("Card failed, or not present");
    tft.setTextColor(ST7735_RED, background_color);
    tft.println("     X");
    tft.setTextColor(ST7735_WHITE, background_color);
    use_sd_card = false;
  }
  else
  {
    tft.println("    OK");
  }
  // ----- Check if SD Card is OK ----- //

  // ----- Wait for WIFI ----- //
  tft.print("WIFI");

  if (WiFi.status() == WL_CONNECTED)
  {
    tft.println("       OK");

    tft.setTextSize(1);
    tft.print("local ip   ");
    tft.println(WiFi.localIP());
    delay(100);

    Serial.println("local ip  ");
    Serial.print(WiFi.localIP());
    Serial.println();
  }
  else
  {
    tft.setTextSize(2);
    tft.setTextColor(ST7735_RED, background_color);
    tft.println("        X");
    tft.setTextColor(ST7735_WHITE, background_color);

    tft.setTextSize(1);
    tft.println();

    tft.println("start without WIFI SELECT");
    tft.println("start ConfigPortal MENU");

    while (left_button_flag == 0 && right_button_flag == 0)
    {
      Serial.println(left_button_flag);
    }

    if (left_button_flag == 1)
    {
      Serial.println("Portal Started");
      tft.println("ConfigPortal started");

      wifiManager.setEnableConfigPortal(true);
      wifiManager.setConfigPortalBlocking(true);

      if (!wifiManager.startConfigPortal("PowerLogger_Portal"))
      {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        // reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
      }

      tft.println();
      tft.print("local ip   ");
      tft.println(WiFi.localIP());
      delay(1000);

      Serial.println("local ip  ");
      Serial.print(WiFi.localIP());
      Serial.println();
      left_button_flag = 0;
    }

    if (right_button_flag == 1)
    {
      Serial.println("getting on without WiFi");
      right_button_flag = 0;
    }
  }
  // ----- Wait for WIFI ----- //

  // ----- Get Time ----- //
  tft.setTextSize(2);
  tft.print("TIME");
  timeClient.begin();
  if (timeClient.update())
  {
    rtc.setTime(timeClient.getEpochTime());
    rtcOffset = millis();
    tft.println("       OK");
  }
  else
  {
    Serial.println("NTP Failed!");
    tft.setTextColor(ST7735_RED, background_color);
    tft.println("        X");
    tft.setTextColor(ST7735_WHITE, background_color);
  }
  // ----- Get Time ----- //

  // ----- Battery check ----- //
  tft.print("BATTERY");
  battery_voltage = get_battery_voltage();
  tft.print("  ");
  if (battery_voltage < 3.3)
  {
    tft.setTextColor(ST7735_RED, background_color);
    tft.println(battery_voltage);
    tft.setTextColor(ST7735_WHITE, background_color);
  }
  else
  {
    tft.println(battery_voltage);
  }
  // ----- Battery check ----- //

  ina3221.setWarnAlertCurrentLimit(INA3221_CH1, -1);
  delay(250);
  ina3221.setCritAlertCurrentLimit(INA3221_CH1, -1);
  delay(500);
  ina3221.setWarnAlertCurrentLimit(INA3221_CH1, 1000);
  delay(250);
  ina3221.setCritAlertCurrentLimit(INA3221_CH1, 1500);
  delay(250);
  tft.setTextSize(1);
  tft.println("             ");
  tft.setTextColor(ST7735_GREEN, background_color);
  tft.setTextSize(2);
  tft.println("BOOT COMPLETE");
  tft.setTextColor(ST7735_WHITE, background_color);
  delay(500);
}

float get_battery_voltage()
{
  float sensorValue = 0;
  for (int i = 1; i <= 5; i++)
  {
    sensorValue = sensorValue + analogRead(0);
  }
  sensorValue = sensorValue / 5;
  return (sensorValue * 5.8) / 4095;
}

void setup_menu()
{

  tft.fillScreen(background_color);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_WHITE, background_color);
  tft.setTextWrap(false);

  while (started == false)
  {
    if (left_button_flag == 1)
    {
      selected = selected + 1;
    }
    if (selected > 5)
    {
      selected = 1;
    }
    if (right_button_flag == 1)
    {
      if (selected == 1)
      {
        use_channel_1 = !use_channel_1;
      }
      else if (selected == 2)
      {
        use_channel_2 = !use_channel_2;
      }
      else if (selected == 3)
      {
        use_channel_3 = !use_channel_3;
      }
      else if (selected == 4)
      {
        selected_avg = selected_avg + 1;
        if (selected_avg > 5)
        {
          selected_avg = 1;
        }
      }
      else if (selected == 5)
      {
        if (setup_error == false)
        {
          started = true;
          ignore_input = true;
        }
      }
    }

    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_RED, background_color);
    tft.print("Setup: ");
    if (setup_error == true)
    {
      tft.print("ERROR");
    }
    else
    {
      tft.print("     ");
    }
    tft.println();
    tft.setTextColor(ST7735_WHITE, background_color);
    tft.println("             ");

    if (selected == 1)
    {
      tft.print(">");
    }
    else
    {
      tft.print(" ");
    }
    tft.print("CH1: ");
    if (use_channel_1)
    {
      tft.println(" ENABLE");
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      tft.setTextColor(ST7735_WHITE, background_color);
    }

    if (selected == 2)
    {
      tft.print(">");
    }
    else
    {
      tft.print(" ");
    }
    tft.print("CH2: ");
    if (use_channel_2)
    {
      tft.println(" ENABLE");
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      tft.setTextColor(ST7735_WHITE, background_color);
    }

    if (selected == 3)
    {
      tft.print(">");
    }
    else
    {
      tft.print(" ");
    }
    tft.print("CH3: ");
    if (use_channel_3)
    {
      tft.println(" ENABLE");
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      tft.setTextColor(ST7735_WHITE, background_color);
    }

    if (selected == 4)
    {
      tft.print(">");
    }
    else
    {
      tft.print(" ");
    }
    tft.print("AVG: ");
    if (selected_avg == 1)
    {
      tft.println("      1");
    }
    else if (selected_avg == 2)
    {
      tft.println("      4");
    }
    else if (selected_avg == 3)
    {
      tft.println("     16");
    }
    else if (selected_avg == 4)
    {
      tft.println("     64");
    }
    else if (selected_avg == 5)
    {
      tft.println("    128");
    }

    tft.println("             ");

    if (selected == 5)
    {
      tft.print(">");
    }
    else
    {
      tft.print(" ");
    }
    tft.println("START");

    if (use_channel_1 == false && use_channel_2 == false && use_channel_3 == false)
    {
      setup_error = true;
    }
    else
    {
      setup_error = false;
    }
    left_button_flag = 0;
    right_button_flag = 0;
    delay(200);
  }

  if (started == true)
  {
    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_RED, background_color);
    tft.println("Setup: ");
    tft.setTextColor(ST7735_WHITE, background_color);
    tft.println("             ");
    tft.print(" CH1: ");
    if (use_channel_1)
    {
      tft.println(" ENABLE");
      ina3221.setChannelEnable(INA3221_CH1);
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      ina3221.setChannelDisable(INA3221_CH1);
      tft.setTextColor(ST7735_WHITE, background_color);
    }
    tft.print(" CH2: ");
    if (use_channel_2)
    {
      tft.println(" ENABLE");
      ina3221.setChannelEnable(INA3221_CH2);
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      ina3221.setChannelDisable(INA3221_CH2);
      tft.setTextColor(ST7735_WHITE, background_color);
    }
    tft.print(" CH3: ");
    if (use_channel_3)
    {
      tft.println(" ENABLE");
      ina3221.setChannelEnable(INA3221_CH3);
    }
    else
    {
      tft.setTextColor(ST7735_RED, background_color);
      tft.println("DISABLE");
      ina3221.setChannelDisable(INA3221_CH3);
      tft.setTextColor(ST7735_WHITE, background_color);
    }
    tft.print(" AVG: ");
    if (selected_avg == 1)
    {
      tft.println("      1");
      ina3221.setAveragingMode(INA3221_REG_CONF_AVG_1);
    }
    else if (selected_avg == 2)
    {
      tft.println("      4");
      ina3221.setAveragingMode(INA3221_REG_CONF_AVG_4);
    }
    else if (selected_avg == 3)
    {
      tft.println("     16");
      ina3221.setAveragingMode(INA3221_REG_CONF_AVG_16);
    }
    else if (selected_avg == 4)
    {
      tft.println("     64");
      ina3221.setAveragingMode(INA3221_REG_CONF_AVG_64);
    }
    else if (selected_avg == 5)
    {
      tft.println("    128");
      ina3221.setAveragingMode(INA3221_REG_CONF_AVG_128);
    }
    tft.println("             ");
    tft.setTextColor(ST7735_GREEN, background_color);
    tft.println(" STARTING ...");
    tft.setTextColor(ST7735_WHITE, background_color);
    if (use_channel_1 == false)
    {
      channel_number = 2;
    }
    if (use_channel_1 == false && use_channel_2 == false)
    {
      channel_number = 3;
    }
    if (use_sd_card == true)
    {
      //  if(file_active == false){
      create_file();
      //  }
    }
    delay(1000);
    ignore_input = false;
  }
}

void handle_left_Interrupt()
{
  display_on_time = millis();
  interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200 && ignore_input == false)
  {
    left_button_flag = 1;
  }
  last_interrupt_time = interrupt_time;
}

void handle_right_Interrupt()
{
  display_on_time = millis();
  interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200 && ignore_input == false)
  {
    right_button_flag = 1;
  }
  last_interrupt_time = interrupt_time;
}

void create_file()
{
  if (use_sd_card == true)
  {

    // file_name = "/" + String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + "_" + String(timeClient.getHours()) + "-" + String(timeClient.getMinutes()) + "-" + String(timeClient.getSeconds()) + ".txt";
    file_name = "/" + String(rtc.getTime("%Y-%B-%d_%H-%M-%S")) + ".txt";
    File Log = SD.open(file_name, FILE_WRITE);
    if (Log)
    {
      Log.print("date");
      Log.print(",");
      Log.print("time");
      if (use_channel_1 == true)
      {
        Log.print(",");
        Log.print("load voltage 1");
        Log.print(",");
        Log.print("current mA 1");
        Log.print(",");
        Log.print("power mW 1");
        Log.print(",");
        Log.print("energy mWh 1");
        Log.print(",");
        Log.print("capacity mAh 1");
      }
      if (use_channel_2 == true)
      {
        Log.print(",");
        Log.print("load voltage 2");
        Log.print(",");
        Log.print("current mA 2");
        Log.print(",");
        Log.print("power mW 2");
        Log.print(",");
        Log.print("energy mWh 2");
        Log.print(",");
        Log.print("capacity mAh 2");
      }
      if (use_channel_3 == true)
      {
        Log.print(",");
        Log.print("load voltage 3");
        Log.print(",");
        Log.print("current mA 3");
        Log.print(",");
        Log.print("power mW 3");
        Log.print(",");
        Log.print("energy mWh 3");
        Log.print(",");
        Log.print("capacity mAh 3");
      }

      Log.println();
      Log.close();
    }

    file_active = true;
  }
}

void write_file()
{
  File Log = SD.open(file_name, FILE_APPEND);
  if (Log)
  {
    // Serial.println(String(rtc.getTime("%Y/%B/%D %H:%M:%S:")) + String((currentMillis-rtcOffset)%1000));
    Log.print(String(rtc.getTime("%y/%m/%d")));
    Log.print(",");
    Log.print(String(rtc.getTime("%H:%M:%S:")) + String((currentMillis - rtcOffset) % 1000));

    if (use_channel_1 == true)
    {
      Log.print(",");
      Log.print(load_voltage_1);
      Log.print(",");
      Log.print(current_mA_1);
      Log.print(",");
      Log.print(load_voltage_1 * current_mA_1);
      Log.print(",");
      Log.print(energy_1);
      Log.print(",");
      Log.print(capacity_1);
    }
    if (use_channel_2 == true)
    {
      Log.print(",");
      Log.print(load_voltage_2);
      Log.print(",");
      Log.print(current_mA_2);
      Log.print(",");
      Log.print(load_voltage_2 * current_mA_2);
      Log.print(",");
      Log.print(energy_2);
      Log.print(",");
      Log.print(capacity_2);
    }
    if (use_channel_3 == true)
    {
      Log.print(",");
      Log.print(load_voltage_3);
      Log.print(",");
      Log.print(current_mA_3);
      Log.print(",");
      Log.print(load_voltage_3 * current_mA_3);
      Log.print(",");
      Log.print(energy_3);
      Log.print(",");
      Log.print(capacity_3);
    }

    Log.println();
    Log.close();
  }
}

void extractIpAddress(char *sourceString, short *ipAddress)
{
  short len = 0;
  char oct[4] = {0}, cnt = 0, cnt1 = 0, i, buf[5];

  len = strlen(sourceString);
  for (i = 0; i < len; i++)
  {
    if (sourceString[i] != '.')
    {
      buf[cnt++] = sourceString[i];
    }
    if (sourceString[i] == '.' || i == len - 1)
    {
      buf[cnt] = '\0';
      cnt = 0;
      oct[cnt1++] = atoi(buf);
    }
  }
  ipAddress[0] = oct[0];
  ipAddress[1] = oct[1];
  ipAddress[2] = oct[2];
  ipAddress[3] = oct[3];
}
