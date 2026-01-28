// ========== Greenhouse Monitor + SD Logger (NO LIGHT SENSOR) ==========
// AHT2x (temp+humidity) + ENS160 (air quality / eCO2) combo board on I2C
// L293D Fan + LCD + RGB LEDs + Alarm + Button UI + SD logging
// Board: Arduino MEGA2560
// I2C: SDA = 20, SCL = 21
// Serial: 9600 baud
//
// SD wiring (MEGA SPI):
//   CS   -> 53
//   SCK  -> 52
//   MOSI -> 51
//   MISO -> 50
//   VCC  -> 5V
//   GND  -> GND
//
// Logs CSV every 10 mins for 24h (144 rows) to GH24H.csv

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <LiquidCrystal.h>

#include <SPI.h>
#include <SD.h>

// -------- LCD (16x2, 4-bit mode) --------
// Wiring: RS->2, E->3, D4->4, D5->5, D6->6, D7->7
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

// -------- AHT2x + ENS160 --------
Adafruit_AHTX0 aht;

// ENS160 addresses: 0x53 or 0x52
DFRobot_ENS160_I2C ens53(&Wire, 0x53);
DFRobot_ENS160_I2C ens52(&Wire, 0x52);
DFRobot_ENS160_I2C* ens = nullptr;
uint8_t ensAddr = 0;

// Sampling period for sensor + Serial (ms)
const unsigned long SAMPLE_MS = 2000;  // 2 seconds
unsigned long lastSampleMs = 0;

// Latest sensor values (used by logic + LCD)
float tcAHT = NAN;   // °C
float rhAHT = NAN;   // %
int   aqi   = -1;    // 1–5
int   tvoc  = -1;    // ppb
int   eco2  = -1;    // ppm

// -------- L293D Fan control --------
#define EN3  45   // Enable channel 3
#define IN3  41   // Direction
#define IN4  40   // Direction

bool fanOn = false;

// -------- LEDs & Buzzer --------
const int RED_LED_PIN   = 36;   // Red: alarm
const int BLUE_LED_PIN  = 37;   // Blue: temp OK
const int GREEN_LED_PIN = 39;   // Green: CO2 OK
const int BUZZER_PIN    = 22;   // Active buzzer

// -------- Buttons (UPDATED PINS) --------
// Previously: MODE=8, UP=9, DOWN=10, MUTE=24
// Now connected to: 24, 23, 25, 27
// Assumed mapping in same order: MODE=24, UP=23, DOWN=25, MUTE=27
const int BUTTON_MODE = 25;   // cycle UI mode
const int BUTTON_UP   = 24;   // increase value
const int BUTTON_DOWN = 23;   // decrease value
const int BUTTON_MUTE = 27;   // mute alarm buzzer

// -------- Setpoints (editable via UI) --------
float TEMP_ON   = 30.0;   // temp above this = out of range (alarm)
int   CO2_MIN_OK = 400;   // ppm
int   CO2_MAX_OK = 1200;  // ppm

// -------- UI modes --------
enum UiMode {
  UI_STATUS = 0,     // show live T/H/CO2
  UI_SET_TEMP = 1,   // adjust TEMP_ON
  UI_SET_CO2_MIN = 2,
  UI_SET_CO2_MAX = 3
};
UiMode uiMode = UI_STATUS;

// Blink / beep timing (ms)
const unsigned long BLUE_BLINK_INTERVAL   = 500;
const unsigned long RED_BLINK_INTERVAL    = 150;
const unsigned long GREEN_BLINK_INTERVAL  = 500;
const unsigned long BUZZER_BEEP_INTERVAL  = 300;

// Blink / beep states
unsigned long lastBlueToggle   = 0;
bool blueState                 = false;

unsigned long lastRedToggle    = 0;
bool redState                  = false;

unsigned long lastGreenToggle  = 0;
bool greenState                = false;

unsigned long lastBuzzerToggle = 0;
bool buzzerState               = false;

// Buzzer mute state
bool buzzerMuted               = false;

// For button edge detection
bool lastModeState  = HIGH;
bool lastUpState    = HIGH;
bool lastDownState  = HIGH;
bool lastMuteState  = HIGH;

// ===== SD logging (24h) =====
const int SD_CS_PIN = 53;
const char* LOG_NAME = "GH24H.csv";

// ✅ UPDATED: log every 10 minutes (instead of 30)
const unsigned long LOG_INTERVAL_MS = 10UL * 60UL * 1000UL;

// For quick testing you can temporarily use:
// const unsigned long LOG_INTERVAL_MS = 5000UL;

unsigned long lastLogMs = 0;
unsigned long logStartMs = 0;
const unsigned long LOG_DURATION_MS = 24UL * 60UL * 60UL * 1000UL;
bool loggingActive = true;

// -------- Fan control functions --------
void fan_start() {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(EN3, HIGH);   // fan ON
}

void fan_stop() {
  digitalWrite(EN3, LOW);    // fan OFF
}

// -------- I2C scan (optional) --------
void scanI2C() {
  Serial.println(F("I2C scan (expect: 0x38=AHT2x, 0x52/0x53=ENS160):"));
  byte count = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found 0x"));
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      count++;
      delay(2);
    }
  }
  if (!count) Serial.println(F("  (no I2C devices found)"));
  Serial.println();
}

// ===== LCD draw function =====
void drawLCD() {
  lcd.clear();

  if (uiMode == UI_STATUS) {
    lcd.setCursor(0, 0);
    lcd.print("T:");
    if (isnan(tcAHT)) lcd.print("--.-");
    else              lcd.print(tcAHT, 1);
    lcd.print("C ");

    lcd.print("H:");
    if (isnan(rhAHT)) lcd.print("--.-");
    else              lcd.print(rhAHT, 1);
    lcd.print('%');

    lcd.setCursor(0, 1);
    lcd.print("CO2:");
    if (ens && eco2 >= 0) {
      lcd.print(eco2);
      lcd.print("ppm");
    } else {
      lcd.print("---ppm");
    }
  }
  else if (uiMode == UI_SET_TEMP) {
    lcd.setCursor(0, 0);
    lcd.print("Set T alarm:");

    lcd.setCursor(0, 1);
    lcd.print(TEMP_ON, 1);
    lcd.print(" C      ");
  }
  else if (uiMode == UI_SET_CO2_MIN) {
    lcd.setCursor(0, 0);
    lcd.print("Set CO2 MIN:");

    lcd.setCursor(0, 1);
    lcd.print(CO2_MIN_OK);
    lcd.print(" ppm    ");
  }
  else if (uiMode == UI_SET_CO2_MAX) {
    lcd.setCursor(0, 0);
    lcd.print("Set CO2 MAX:");

    lcd.setCursor(0, 1);
    lcd.print(CO2_MAX_OK);
    lcd.print(" ppm    ");
  }
}

// ===== SD helper functions =====
void sdInitOrWarn() {
  // CRITICAL: MEGA SPI master pin must be OUTPUT
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);

  // CS pin output + deselect
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  Serial.println(F("Initializing SD..."));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("ERR: SD init failed. Check wiring + FAT32 card."));
    lcd.setCursor(0, 1);
    lcd.print("SD ERR         ");
  } else {
    Serial.println(F("SD OK"));

    // Write header once
    if (!SD.exists(LOG_NAME)) {
      File f = SD.open(LOG_NAME, FILE_WRITE);
      if (f) {
        f.println("Timestamp,Temperature (C),Humidity (%),CO2 (ppm),Air Quality,TVOC,Fan ON/OFF");
        f.close();
        Serial.println(F("CSV header written."));
      } else {
        Serial.println(F("ERR: couldn't create CSV file."));
      }
    } else {
      Serial.println(F("CSV exists (header already done)."));
    }
  }
}

// Timestamp without RTC: seconds since boot
String timestampNoRTC() {
  unsigned long sec = millis() / 1000UL;
  return String(sec);
}

void appendCsvRow() {
  File f = SD.open(LOG_NAME, FILE_WRITE);
  if (!f) {
    Serial.println(F("ERR: couldn't open GH24H.csv for writing"));
    return;
  }

  // Timestamp
  f.print(timestampNoRTC()); f.print(',');

  // Temperature
  if (isnan(tcAHT)) f.print("NA");
  else f.print(tcAHT, 1);
  f.print(',');

  // Humidity
  if (isnan(rhAHT)) f.print("NA");
  else f.print(rhAHT, 1);
  f.print(',');

  // CO2
  if (ens && eco2 >= 0) f.print(eco2);
  else f.print("NA");
  f.print(',');

  // Air Quality (AQI)
  if (ens && aqi >= 0) f.print(aqi);
  else f.print("NA");
  f.print(',');

  // TVOC
  if (ens && tvoc >= 0) f.print(tvoc);
  else f.print("NA");
  f.print(',');

  // Fan state
  f.println(fanOn ? "ON" : "OFF");

  f.close();
  Serial.println(F("SD: Logged row"));
}

// ================== setup ==================
void setup() {
  Serial.begin(9600);
  Wire.begin();  // MEGA: SDA=20, SCL=21

  // ---- Init LCD ----
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Booting...");
  lcd.setCursor(0, 1);
  lcd.print("Setup start");

  // ---- Pins ----
  pinMode(EN3, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  fan_stop();

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_MUTE, INPUT_PULLUP);
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  pinMode(BUTTON_UP,   INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  Serial.println(F("\n==== AHT2x + ENS160 + Fan (L293D) + LCD + SD ===="));
  scanI2C();

  // ---- Init AHT2x ----
  if (!aht.begin()) {
    Serial.println(F("ERR: AHT2x not found (0x38), check wiring."));
    lcd.setCursor(0, 1);
    lcd.print("AHT ERR       ");
  } else {
    Serial.println(F("AHT2x OK"));
  }

  // ---- Init ENS160 (DFRobot) ----
  if (ens53.begin() == 0) {
    ens = &ens53;
    ensAddr = 0x53;
  } else if (ens52.begin() == 0) {
    ens = &ens52;
    ensAddr = 0x52;
  }

  if (ens) {
    Serial.print(F("ENS160 OK, address: 0x"));
    Serial.println(ensAddr, HEX);
  } else {
    Serial.println(F("ERR: ENS160 not found at 0x53/0x52."));
    lcd.setCursor(0, 1);
    lcd.print("ENS ERR       ");
  }

  // ---- Init SD ----
  sdInitOrWarn();
  logStartMs = millis();
  lastLogMs  = millis();

  delay(500);
  drawLCD(); // draw initial screen
  Serial.println();
  Serial.println(F("Time(s)\tTemp(C)\tHum(%)\tAQI\tTVOC(ppb)\tCO2(ppm)\tFan"));
}

// ================== loop ==================
void loop() {
  unsigned long now = millis();

  // ===== 24h SD logging scheduler =====
  if (loggingActive) {
    if (now - logStartMs >= LOG_DURATION_MS) {
      loggingActive = false;
      Serial.println(F("24h logging complete."));
    } else if (now - lastLogMs >= LOG_INTERVAL_MS) {
      lastLogMs = now;
      appendCsvRow();
    }
  }

  // -------- A) Periodic sensor read + Serial + LCD (every SAMPLE_MS) --------
  if (now - lastSampleMs >= SAMPLE_MS) {
    lastSampleMs = now;

    unsigned long sec = now / 1000;

    // 1. Read AHT2x
    sensors_event_t h_aht, t_aht;
    if (aht.getEvent(&h_aht, &t_aht)) {
      tcAHT = t_aht.temperature;
      rhAHT = h_aht.relative_humidity;
    } else {
      tcAHT = NAN;
      rhAHT = NAN;
    }

    // 2. Read ENS160
    if (ens) {
      if (!isnan(tcAHT) && !isnan(rhAHT)) {
        ens->setTempAndHum(tcAHT, rhAHT);
      }
      aqi  = ens->getAQI();
      tvoc = ens->getTVOC();
      eco2 = ens->getECO2();
    } else {
      aqi  = -1;
      tvoc = -1;
      eco2 = -1;
    }

    // 3. Serial output
    Serial.print(sec);        Serial.print('\t');

    if (isnan(tcAHT)) Serial.print("NA");
    else              Serial.print(tcAHT, 1);
    Serial.print('\t');

    if (isnan(rhAHT)) Serial.print("NA");
    else              Serial.print(rhAHT, 1);
    Serial.print('\t');

    if (ens) {
      Serial.print(aqi);  Serial.print('\t');
      Serial.print(tvoc); Serial.print('\t');
      Serial.print(eco2); Serial.print('\t');
    } else {
      Serial.print("NA\tNA\tNA\t");
    }

    Serial.println(fanOn ? F("ON") : F("OFF"));

    // Update LCD after fresh readings
    drawLCD();
  }

  // -------- B) Read buttons (UI + mute) --------
  bool modeState  = digitalRead(BUTTON_MODE);
  bool upState    = digitalRead(BUTTON_UP);
  bool downState  = digitalRead(BUTTON_DOWN);
  bool muteState  = digitalRead(BUTTON_MUTE);

  bool changedUI = false;

  // MODE button: change UI mode on falling edge
  if (lastModeState == HIGH && modeState == LOW) {
    uiMode = (UiMode)(((int)uiMode + 1) % 4);
    changedUI = true;
  }
  lastModeState = modeState;

  // UP button
  if (lastUpState == HIGH && upState == LOW) {
    if (uiMode == UI_SET_TEMP)          TEMP_ON += 0.5;
    else if (uiMode == UI_SET_CO2_MIN)  CO2_MIN_OK += 50;
    else if (uiMode == UI_SET_CO2_MAX)  CO2_MAX_OK += 50;
    changedUI = true;
  }

  // DOWN button
  if (lastDownState == HIGH && downState == LOW) {
    if (uiMode == UI_SET_TEMP)          TEMP_ON -= 0.5;
    else if (uiMode == UI_SET_CO2_MIN)  CO2_MIN_OK -= 50;
    else if (uiMode == UI_SET_CO2_MAX)  CO2_MAX_OK -= 50;
    changedUI = true;
  }

  lastUpState   = upState;
  lastDownState = downState;

  // Clamp ranges
  if (TEMP_ON < 10.0) TEMP_ON = 10.0;
  if (TEMP_ON > 50.0) TEMP_ON = 50.0;

  if (CO2_MIN_OK < 0) CO2_MIN_OK = 0;
  if (CO2_MIN_OK > CO2_MAX_OK - 50) CO2_MIN_OK = CO2_MAX_OK - 50;
  if (CO2_MAX_OK < CO2_MIN_OK + 50) CO2_MAX_OK = CO2_MIN_OK + 50;
  if (CO2_MAX_OK > 5000) CO2_MAX_OK = 5000;

  // Instant LCD refresh when user changes settings
  if (changedUI) {
    drawLCD();
  }

  // -------- C) Evaluate conditions (temp / CO2) --------
  bool tempOk   = (!isnan(tcAHT) && tcAHT < TEMP_ON);
  bool co2Valid = (ens && eco2 >= 0);
  bool co2Ok    = co2Valid && eco2 >= CO2_MIN_OK && eco2 <= CO2_MAX_OK;

  // Red alert if temperature high OR CO2 out of range
  bool redAlert = (!tempOk) || (co2Valid && !co2Ok);

  // Fan follows red alert
  if (redAlert && !fanOn) {
    fanOn = true;
    fan_start();
  } else if (!redAlert && fanOn) {
    fanOn = false;
    fan_stop();
  }

  // MUTE button: if pressed during alarm, mute buzzer
  if (lastMuteState == HIGH && muteState == LOW && redAlert) {
    buzzerMuted = true;
  }
  lastMuteState = muteState;

  // When everything normal again, auto-unmute
  if (!redAlert) {
    buzzerMuted = false;
  }

  // -------- D) LEDs --------

  // Blue LED: temp OK
  if (tempOk) {
    if (now - lastBlueToggle >= BLUE_BLINK_INTERVAL) {
      lastBlueToggle = now;
      blueState = !blueState;
      digitalWrite(BLUE_LED_PIN, blueState ? HIGH : LOW);
    }
  } else {
    blueState = false;
    digitalWrite(BLUE_LED_PIN, LOW);
  }

  // Green LED: CO2 OK
  if (co2Ok) {
    if (now - lastGreenToggle >= GREEN_BLINK_INTERVAL) {
      lastGreenToggle = now;
      greenState = !greenState;
      digitalWrite(GREEN_LED_PIN, greenState ? HIGH : LOW);
    }
  } else {
    greenState = false;
    digitalWrite(GREEN_LED_PIN, LOW);
  }

  // Red LED: any alarm
  if (redAlert) {
    if (now - lastRedToggle >= RED_BLINK_INTERVAL) {
      lastRedToggle = now;
      redState = !redState;
      digitalWrite(RED_LED_PIN, redState ? HIGH : LOW);
    }
  } else {
    redState = false;
    digitalWrite(RED_LED_PIN, LOW);
  }

  // -------- E) Buzzer --------
  if (redAlert && !buzzerMuted) {
    if (now - lastBuzzerToggle >= BUZZER_BEEP_INTERVAL) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  } else {
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}
