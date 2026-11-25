#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "PlatformIO must build this application in Zigbee End Device mode"
#endif

#include "Zigbee.h"
#include <AHT20.h>

// ─────────────────────────────────────────────────────────────
// ENS160 CO2 sensor (ScioSense, I2C 0x53)
// Register map:
//   0x00  PART_ID            (read-only, = 0x0160)
//   0x10  OPMODE             0x00=Deep Sleep, 0x01=Idle,
//                             0x02=Standard, 0xF0=Soft Reset
//   0x13  TEMP_IN            (2 bytes, little-endian, 1/64 K)
//   0x15  RH_IN              (2 bytes, little-endian, 1/512 %RH)
//   0x20  DEVICE_STATUS      bit[1]=NEWDAT, bit[3:2]=VALIDITY
//   0x24  DATA_ECO2         (2 bytes, little-endian, ppm)
//
// Measurement cycle:
//   1. Set OPMODE = 0x02 (Standard) — heater starts, auto measurement
//   2. Wait ~3 min for warm-up
//   3. NEWDAT flag toggles after each 1s measurement
//   4. Read DATA_ECO2 when NEWDAT=1
// ──────��──────────────────────────────────────────────────────
constexpr uint8_t ENS160_ADDR = 0x53;
constexpr uint8_t ENS160_REG_PART_ID       = 0x00;
constexpr uint8_t ENS160_REG_OPMODE        = 0x10;
constexpr uint8_t ENS160_REG_TEMP_IN       = 0x13;
constexpr uint8_t ENS160_REG_RH_IN         = 0x15;
constexpr uint8_t ENS160_REG_DEVICE_STATUS = 0x20;
constexpr uint8_t ENS160_REG_DATA_ECO2     = 0x24;

constexpr uint8_t ENS160_OPMODE_STD        = 0x02;
constexpr uint8_t ENS160_OPMODE_RESET      = 0xF0;
constexpr uint16_t ENS160_PART_ID = 0x0160;

constexpr uint8_t RGB_LED_PIN = 8;
constexpr uint32_t BLINK_INTERVAL_MS = 500;
constexpr uint32_t REPORT_INTERVAL_MS = 10000;
constexpr uint32_t AHT_SAMPLE_INTERVAL_MS = 5000;
constexpr uint32_t ENS160_READ_INTERVAL_MS = 1000;

ZigbeeTempSensor climateSensor(10);
ZigbeeCarbonDioxideSensor carbonDioxideSensor(11);

// ENS160 state
static uint32_t lastEns160Read = 0;
static uint16_t co2Value = 0;
static bool co2Valid = false;
static uint8_t ens160Validity = 0;

// AHT values
static float temperature = 0.0f;
static float humidity = 0.0f;
static bool ahtValid = false;
static uint32_t lastAhtSample = 0;

#include <Wire.h>

// ── ENS160 register helpers ─────────────────────────────────
void ens160_write_reg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t ens160_read_reg8(uint8_t reg) {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ENS160_ADDR, (uint8_t)1);
  return Wire.available() ? (uint8_t)Wire.read() : 0xFF;
}

void ens160_feed_env_data(float tempC, float rhPercent) {
  uint16_t temp_raw = (uint16_t)((tempC + 273.15f) * 64.0f);
  uint16_t rh_raw   = (uint16_t)(rhPercent * 512.0f);
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_REG_TEMP_IN);
  Wire.write(temp_raw & 0xFF);
  Wire.write((temp_raw >> 8) & 0xFF);
  Wire.write(rh_raw & 0xFF);
  Wire.write((rh_raw >> 8) & 0xFF);
  Wire.endTransmission();
}

bool ens160_read_co2(uint16_t& co2) {
  uint8_t status = ens160_read_reg8(ENS160_REG_DEVICE_STATUS);
  ens160Validity = (status >> 2) & 0x03;

  if (!(status & 0x02)) return false;

  uint16_t lo = 0xFF00, hi = 0xFF;
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_REG_DATA_ECO2);
  Wire.endTransmission(false);
  Wire.requestFrom(ENS160_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    lo = Wire.read();
    hi = Wire.read();
  }
  co2 = lo | ((uint16_t)hi << 8);
  return true;
}

bool ens160_init() {
  ens160_write_reg8(ENS160_REG_OPMODE, ENS160_OPMODE_RESET);
  delay(50);

  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_REG_PART_ID);
  Wire.endTransmission(false);
  Wire.requestFrom(ENS160_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return false;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  uint16_t part_id = lo | ((uint16_t)hi << 8);
  if (part_id != ENS160_PART_ID) {
    Serial.printf("ENS160 Part ID mismatch: 0x%04X\n", part_id);
    return false;
  }

  ens160_write_reg8(ENS160_REG_OPMODE, ENS160_OPMODE_STD);
  delay(50);
  Serial.println("ENS160 initialized (Standard Mode)");
  return true;
}

// ── setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

  Wire.begin(5, 6, 200000);
  delay(100);

  if (!aht.begin()) {
    Serial.println("AHT21 init failed");
  } else {
    Serial.println("AHT21 initialized");
  }

  bool ens160_ok = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    Wire.beginTransmission(ENS160_ADDR);
    if (Wire.endTransmission() == 0) {
      ens160_ok = true;
      break;
    }
    delay(500);
  }
  if (ens160_ok) {
    ens160_init();
  } else {
    Serial.println("ENS160 not found");
  }

  climateSensor.setManufacturerAndModel("DIY", "ESP32C6-Climate-CO2");
  climateSensor.setMinMaxValue(-10, 60);
  climateSensor.setDefaultValue(24);
  climateSensor.setTolerance(0.1);
  climateSensor.addHumiditySensor(0, 100, 0.1, 50);

  carbonDioxideSensor.setManufacturerAndModel("DIY", "ESP32C6-Climate-CO2");
  carbonDioxideSensor.setMinMaxValue(0, 10000);

  if (!Zigbee.addEndpoint(&climateSensor) ||
      !Zigbee.addEndpoint(&carbonDioxideSensor)) {
    Serial.println("Failed to register Zigbee sensor endpoints");
    ESP.restart();
  }

  if (!Zigbee.begin()) {
    Serial.println("Failed to start Zigbee");
    ESP.restart();
  }

  Serial.println("Waiting for Zigbee network...");
  while (!Zigbee.connected()) {
    delay(100);
  }
  Serial.println("Connected to Zigbee network");
}

// ── loop ────────────────────────────────────────────────────���
void loop() {
  static uint32_t lastLedToggle = 0;
  static uint32_t lastReport = 0;
  static bool ledOn = false;
  static bool firstReport = true;
  static uint32_t lastCo2Log = 0;
  const uint32_t now = millis();

  // LED blink
  if (now - lastLedToggle >= BLINK_INTERVAL_MS) {
    lastLedToggle = now;
    ledOn = !ledOn;
    rgbLedWrite(RGB_LED_PIN, 0, 0, ledOn ? 16 : 0);
  }

  // AHT21: sample every 5s
  if (!ahtValid || now - lastAhtSample >= AHT_SAMPLE_INTERVAL_MS) {
    lastAhtSample = now;
    temperature = aht.getTemperature();
    humidity = aht.getHumidity();
    ahtValid = true;
    if (ahtValid) {
      ens160_feed_env_data(temperature, humidity);
    }
  }

  // ENS160: read CO2 every 1s
  if (now - lastEns160Read >= ENS160_READ_INTERVAL_MS) {
    lastEns160Read = now;
    uint16_t newCo2;
    if (ens160_read_co2(newCo2)) {
      co2Value = newCo2;
      co2Valid = (ens160Validity == 0);
      if (now - lastCo2Log >= 30000) {
        lastCo2Log = now;
        Serial.printf("ENS160 valid=%d co2=%u ppm\n",
                      co2Valid ? 1 : 0, co2Value);
      }
    }
  }

  // Report to Zigbee every 10s
  if (firstReport || now - lastReport >= REPORT_INTERVAL_MS) {
    firstReport = false;
    lastReport = now;

    climateSensor.setTemperature(ahtValid ? temperature : 24.0f);
    climateSensor.setHumidity(ahtValid ? humidity : 50.0f);

    if (co2Valid) {
      carbonDioxideSensor.setCarbonDioxide((float)co2Value);
      carbonDioxideSensor.report();
    }

    climateSensor.report();

    Serial.printf("T:%.1f C  H:%.1f%%  CO2:%s\n",
                  temperature, humidity,
                  co2Valid ? String(co2Value).c_str() : "N/A");
  }

  delay(10);
}
