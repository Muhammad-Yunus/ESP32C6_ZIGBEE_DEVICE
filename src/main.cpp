#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#ifndef ZIGBEE_MODE_ED
#error "PlatformIO must build this application in Zigbee End Device mode"
#endif

#include <AHT20.h>
#include "ZigbeeClimateCO2Sensor.h"

// ── Hardware Pins & Timing ───────────────────────────────────
constexpr uint8_t I2C_SDA_PIN = 5;
constexpr uint8_t I2C_SCL_PIN = 6;
constexpr uint32_t I2C_FREQ_HZ = 100000;
constexpr uint8_t RGB_LED_PIN = 8;
constexpr uint8_t BOOT_BUTTON_PIN = 9;
constexpr uint32_t FACTORY_RESET_HOLD_MS = 3000;

constexpr uint32_t AHT_SAMPLE_INTERVAL_MS = 5000;
constexpr uint32_t ENS160_READ_INTERVAL_MS = 1000;
constexpr uint32_t REPORT_INTERVAL_MS = 10000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 60000;

constexpr float TEMP_REPORT_DELTA = 0.2f;
constexpr float HUM_REPORT_DELTA = 0.5f;
constexpr float CO2_REPORT_DELTA = 25.0f;

// ── ENS160 Register Map (ScioSense, I2C 0x53) ────────────────
constexpr uint8_t ENS160_ADDR = 0x53;
constexpr uint8_t ENS160_REG_PART_ID = 0x00;
constexpr uint8_t ENS160_REG_OPMODE = 0x10;
constexpr uint8_t ENS160_REG_TEMP_IN = 0x13;
constexpr uint8_t ENS160_REG_DEVICE_STATUS = 0x20;
constexpr uint8_t ENS160_REG_DATA_ECO2 = 0x24;

constexpr uint8_t ENS160_OPMODE_STD = 0x02;
constexpr uint8_t ENS160_OPMODE_RESET = 0xF0;
constexpr uint16_t ENS160_PART_ID = 0x0160;

// ── Sensors & Zigbee Endpoint ────────────────────────────────
AHT20 aht;
ZigbeeClimateCO2Sensor sensor(10);

// ── Sensor State ─────────────────────────────────────────────
static bool ens160Ready = false;
static uint32_t lastEns160Read = 0;
static uint16_t co2Value = 0;
static bool co2Valid = false;
static uint8_t ens160Validity = 3;

static float temperature = NAN;
static float humidity = NAN;
static bool ahtReady = false;
static bool ahtValid = false;
static uint32_t lastAhtSample = 0;

// ── Reporting State ──────────────────────────────────────────
static bool wasConnected = false;
static float lastReportedTemp = NAN;
static float lastReportedHum = NAN;
static float lastReportedCo2 = NAN;
static uint32_t lastTempReportMs = 0;
static uint32_t lastHumReportMs = 0;
static uint32_t lastCo2ReportMs = 0;

const char *ens160ValidityLabel(uint8_t validity) {
  switch (validity) {
    case 0:
      return "Normal";
    case 1:
      return "WarmUp";
    case 2:
      return "InitialStartUp";
    case 3:
      return "Invalid";
    default:
      return "Unknown";
  }
}

bool ens160_write_reg8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool ens160_read_reg8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(ENS160_ADDR, (uint8_t)1) < 1) {
    return false;
  }
  val = (uint8_t)Wire.read();
  return true;
}

bool ens160_feed_env_data(float tempC, float rhPercent) {
  if (!ens160Ready) {
    return false;
  }
  uint16_t temp_raw = (uint16_t)((tempC + 273.15f) * 64.0f);
  uint16_t rh_raw = (uint16_t)(rhPercent * 512.0f);
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_REG_TEMP_IN);
  Wire.write(temp_raw & 0xFF);
  Wire.write((temp_raw >> 8) & 0xFF);
  Wire.write(rh_raw & 0xFF);
  Wire.write((rh_raw >> 8) & 0xFF);
  return Wire.endTransmission() == 0;
}

bool ens160_read_co2(uint16_t &co2) {
  if (!ens160Ready) {
    return false;
  }
  uint8_t status = 0;
  if (!ens160_read_reg8(ENS160_REG_DEVICE_STATUS, status)) {
    return false;
  }
  ens160Validity = (status >> 2) & 0x03;

  // bit[1] = NEWDAT
  if (!(status & 0x02)) {
    return false;
  }

  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_REG_DATA_ECO2);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(ENS160_ADDR, (uint8_t)2) < 2) {
    return false;
  }
  uint8_t lo = (uint8_t)Wire.read();
  uint8_t hi = (uint8_t)Wire.read();
  co2 = (uint16_t)lo | ((uint16_t)hi << 8);
  return true;
}

bool ens160_init() {
  for (int attempt = 0; attempt < 5; attempt++) {
    Wire.beginTransmission(ENS160_ADDR);
    if (Wire.endTransmission() == 0) {
      if (!ens160_write_reg8(ENS160_REG_OPMODE, ENS160_OPMODE_RESET)) {
        return false;
      }
      delay(50);

      Wire.beginTransmission(ENS160_ADDR);
      Wire.write(ENS160_REG_PART_ID);
      if (Wire.endTransmission(false) != 0) {
        return false;
      }
      if (Wire.requestFrom(ENS160_ADDR, (uint8_t)2) < 2) {
        return false;
      }
      uint8_t lo = (uint8_t)Wire.read();
      uint8_t hi = (uint8_t)Wire.read();
      uint16_t part_id = (uint16_t)lo | ((uint16_t)hi << 8);
      if (part_id != ENS160_PART_ID) {
        Serial.printf("ENS160 at 0x%02X Part ID mismatch: 0x%04X\n", ENS160_ADDR, part_id);
        return false;
      }

      if (!ens160_write_reg8(ENS160_REG_OPMODE, ENS160_OPMODE_STD)) {
        return false;
      }
      delay(50);
      ens160Ready = true;
      Serial.printf("ENS160 initialized at 0x%02X (Standard Mode)\n", ENS160_ADDR);
      return true;
    }
    delay(300);
  }
  Serial.printf("ENS160 not found at 0x%02X\n", ENS160_ADDR);
  return false;
}

void handleFactoryResetButton(uint32_t now) {
  static uint32_t pressStartMs = 0;
  if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
    if (pressStartMs == 0) {
      pressStartMs = now;
    } else if (now - pressStartMs >= FACTORY_RESET_HOLD_MS) {
      Serial.println("Factory resetting Zigbee NVRAM...");
      rgbLedWrite(RGB_LED_PIN, 32, 0, 0);
      delay(200);
      Zigbee.factoryReset(true);
    }
  } else {
    pressStartMs = 0;
  }
}

void updateStatusLed(uint32_t now, bool connected) {
  static uint32_t lastLedToggle = 0;
  static bool ledOn = false;
  const uint32_t interval = connected ? 1000 : 250;
  if (now - lastLedToggle < interval) {
    return;
  }
  lastLedToggle = now;
  ledOn = !ledOn;
  if (!ledOn) {
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
    return;
  }
  if (!connected) {
    rgbLedWrite(RGB_LED_PIN, 0, 0, 16);
  } else if (co2Valid) {
    rgbLedWrite(RGB_LED_PIN, 0, 16, 0);
  } else {
    rgbLedWrite(RGB_LED_PIN, 12, 8, 0);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(100);

  ahtReady = aht.begin();
  Serial.println(ahtReady ? "AHT21 initialized" : "AHT21 init failed");

  ens160_init();

  sensor.setManufacturerAndModel("DIY", "ESP32C6-Climate-CO2");
  sensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
  sensor.setMinMaxValue(-10.0f, 60.0f);
  sensor.setTolerance(0.1f);
  sensor.addHumiditySensor(0.0f, 100.0f, 0.1f);
  sensor.addCarbonDioxideSensor(0.0f, 65000.0f, 0.0f);

  if (!Zigbee.addEndpoint(&sensor)) {
    Serial.println("Failed to register Zigbee endpoint 10");
    ESP.restart();
  }

  Zigbee.setRxOnWhenIdle(true);

  if (!Zigbee.begin(ZIGBEE_END_DEVICE)) {
    Serial.println("Failed to start Zigbee");
    ESP.restart();
  }
  Serial.println("Zigbee started (End Device, EP 10), waiting for network...");
}

void loop() {
  const uint32_t now = millis();
  handleFactoryResetButton(now);

  const bool connected = Zigbee.connected();
  if (connected && !wasConnected) {
    wasConnected = true;
    lastReportedTemp = NAN;
    lastReportedHum = NAN;
    lastReportedCo2 = NAN;
    Serial.println("Connected to Zigbee network");
  } else if (!connected && wasConnected) {
    wasConnected = false;
    Serial.println("Disconnected from Zigbee network");
  }

  updateStatusLed(now, connected);

  // Sample AHT21 every 5s
  if (ahtReady && (!ahtValid || now - lastAhtSample >= AHT_SAMPLE_INTERVAL_MS)) {
    lastAhtSample = now;
    float t = aht.getTemperature();
    float h = aht.getHumidity();
    if (!isnan(t) && !isnan(h) && t >= -40.0f && t <= 85.0f && h >= 0.0f && h <= 100.0f) {
      temperature = t;
      humidity = h;
      ahtValid = true;
      ens160_feed_env_data(temperature, humidity);
    }
  }

  // Poll ENS160 every 1s
  if (ens160Ready && now - lastEns160Read >= ENS160_READ_INTERVAL_MS) {
    lastEns160Read = now;
    uint16_t newCo2 = 0;
    if (ens160_read_co2(newCo2)) {
      co2Value = newCo2;
      co2Valid = (ens160Validity == 0 && newCo2 >= 400 && newCo2 <= 65000);
    }
  }

  // Telemetry & Zigbee reporting cadence (every 10s)
  static uint32_t lastTick = 0;
  static bool firstTick = true;
  if (firstTick || now - lastTick >= REPORT_INTERVAL_MS) {
    firstTick = false;
    lastTick = now;

    if (connected) {
      if (ahtValid) {
        sensor.setTemperature(temperature);
        sensor.setHumidity(humidity);

        if (isnan(lastReportedTemp) ||
            fabsf(temperature - lastReportedTemp) >= TEMP_REPORT_DELTA ||
            (now - lastTempReportMs >= HEARTBEAT_INTERVAL_MS)) {
          if (sensor.reportTemperature()) {
            lastReportedTemp = temperature;
            lastTempReportMs = now;
          }
        }

        if (isnan(lastReportedHum) ||
            fabsf(humidity - lastReportedHum) >= HUM_REPORT_DELTA ||
            (now - lastHumReportMs >= HEARTBEAT_INTERVAL_MS)) {
          if (sensor.reportHumidity()) {
            lastReportedHum = humidity;
            lastHumReportMs = now;
          }
        }
      }

      if (co2Valid) {
        float co2Float = (float)co2Value;
        sensor.setCarbonDioxide(co2Float);

        if (isnan(lastReportedCo2) ||
            fabsf(co2Float - lastReportedCo2) >= CO2_REPORT_DELTA ||
            (now - lastCo2ReportMs >= HEARTBEAT_INTERVAL_MS)) {
          if (sensor.reportCarbonDioxide()) {
            lastReportedCo2 = co2Float;
            lastCo2ReportMs = now;
          }
        }
      }
    }

    char tempBuf[16];
    char humBuf[16];
    if (ahtValid) {
      snprintf(tempBuf, sizeof(tempBuf), "%.1f C", temperature);
      snprintf(humBuf, sizeof(humBuf), "%.1f%%", humidity);
    } else {
      snprintf(tempBuf, sizeof(tempBuf), "N/A");
      snprintf(humBuf, sizeof(humBuf), "N/A");
    }

    if (!ens160Ready) {
      Serial.printf("[ZB:%s] T:%s  H:%s  CO2:N/A(ens160=off)\n",
                    connected ? "OK" : "WAIT", tempBuf, humBuf);
    } else if (co2Valid) {
      Serial.printf("[ZB:%s] T:%s  H:%s  CO2:%u ppm (validity=%u:%s)\n",
                    connected ? "OK" : "WAIT", tempBuf, humBuf,
                    co2Value, ens160Validity, ens160ValidityLabel(ens160Validity));
    } else {
      Serial.printf("[ZB:%s] T:%s  H:%s  CO2:warming(raw=%u, validity=%u:%s)\n",
                    connected ? "OK" : "WAIT", tempBuf, humBuf,
                    co2Value, ens160Validity, ens160ValidityLabel(ens160Validity));
    }
  }

  delay(10);
}
