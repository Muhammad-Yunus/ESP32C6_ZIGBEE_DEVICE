#include <Arduino.h>

#ifndef ZIGBEE_MODE_ED
#error "PlatformIO must build this application in Zigbee End Device mode"
#endif

#include "Zigbee.h"
#include <AHT20.h>

// AHT21 (I2C 0x38)
AHT20 aht;

// ENS160 CO2 sensor (I2C 0x53)
constexpr uint8_t ENS160_ADDR = 0x53;
constexpr uint8_t ENS160_CMD_MEASURE_TRIGGER = 0x00;
constexpr uint8_t ENS160_CMD_READ_RESULT = 0x04;
constexpr uint8_t ENS160_CMD_SOFT_RESET = 0x48;

constexpr uint8_t RGB_LED_PIN = 8;
constexpr uint32_t BLINK_INTERVAL_MS = 500;
constexpr uint32_t REPORT_INTERVAL_MS = 10000;
constexpr uint32_t ENS160_SAMPLE_INTERVAL_MS = 1000;
constexpr uint32_t AHT_SAMPLE_INTERVAL_MS = 5000;

ZigbeeTempSensor climateSensor(10);
ZigbeeCarbonDioxideSensor carbonDioxideSensor(11);
bool automaticReportingConfigured = false;

// ENS160 measurement state
static uint32_t lastEns160Measure = 0;
static uint16_t co2Value = 0;
static float co2Accuracy = 0.0f;
static bool co2Valid = false;

// AHT values
static float temperature = 0.0f;
static float humidity = 0.0f;
static bool ahtValid = false;
static uint32_t lastAhtSample = 0;

// I2C helpers for ENS160
#include <Wire.h>

void ens160_trigger_measurement() {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_CMD_MEASURE_TRIGGER);
  Wire.endTransmission();
}

void ens160_soft_reset() {
  Wire.beginTransmission(ENS160_ADDR);
  Wire.write(ENS160_CMD_SOFT_RESET);
  Wire.endTransmission();
  delay(3000); // ENS160 needs ~3s to complete soft reset
}

bool ens160_read_result(uint16_t& co2, float& accuracy) {
  uint8_t buf[8];
  Wire.requestFrom(ENS160_ADDR, 8);
  if (Wire.available() < 8) return false;

  for (int i = 0; i < 8; i++) buf[i] = Wire.read();

  // CO2 value: bytes 2-3 (big-endian)
  co2 = (buf[2] << 8) | buf[3];
  // Accuracy: bytes 4-5 (big-endian, 0.001 precision)
  uint16_t acc_raw = (buf[4] << 8) | buf[5];
  accuracy = acc_raw / 1000.0f;

  return true;
}

void setup() {
  Serial.begin(115200);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

  // Init I2C
  Wire.begin(5, 6, 200000); // SDA=GPIO5, SCL=GPIO6, 200kHz
  delay(100);

  // Init AHT21
  if (!aht.begin()) {
    Serial.println("AHT21 init failed");
  } else {
    Serial.println("AHT21 initialized");
  }

  // Probe ENS160
  bool ens160_found = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(ENS160_ADDR);
    if (Wire.endTransmission() == 0) { ens160_found = true; break; }
    delay(500);
  }
  if (ens160_found) {
    Serial.println("ENS160 detected, sending soft reset...");
    ens160_soft_reset();
    Serial.println("ENS160 ready");
  } else {
    Serial.println("ENS160 not found - CO2 will report 0");
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

  Serial.println("Starting Zigbee End Device...");
  if (!Zigbee.begin()) {
    Serial.println("Failed to start Zigbee");
    ESP.restart();
  }

  Serial.println("Waiting for Zigbee network...");
  while (!Zigbee.connected()) {
    delay(100);
  }
  Serial.println("Connected to Zigbee network");

  const bool temperatureReportingConfigured =
    climateSensor.setReporting(0, 10, 0);
  const bool humidityReportingConfigured =
    climateSensor.setHumidityReporting(0, 10, 0);
  const bool co2ReportingConfigured =
    carbonDioxideSensor.setReporting(0, 10, 0);

  automaticReportingConfigured = temperatureReportingConfigured &&
                                humidityReportingConfigured &&
                                co2ReportingConfigured;
  if (!automaticReportingConfigured) {
    Serial.println("Automatic Zigbee reporting setup failed; using explicit reports");
  } else {
    Serial.println("Automatic Zigbee reporting configured for 10 seconds");
  }

  // Trigger first ENS160 measurement
  if (ens160_found) {
    ens160_trigger_measurement();
    lastEns160Measure = millis();
  }
}

void loop() {
  static uint32_t lastLedToggle = 0;
  static uint32_t lastReport = 0;
  static bool ledOn = false;
  static bool firstReport = true;
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
  }

  // ENS160: trigger every 1s, read previous result
  if (now - lastEns160Measure >= ENS160_SAMPLE_INTERVAL_MS) {
    ens160_trigger_measurement();
    lastEns160Measure = now;
    // Read result from previous trigger (ENS160 returns last completed reading)
    uint16_t newCo2;
    float newAcc;
    if (ens160_read_result(newCo2, newAcc) && newCo2 > 0 && newCo2 < 20000) {
      co2Value = newCo2;
      co2Accuracy = newAcc;
      co2Valid = true;
    }
  }

  // Report to Zigbee every 10s
  if (firstReport || now - lastReport >= REPORT_INTERVAL_MS) {
    firstReport = false;
    lastReport = now;

    const bool temperatureUpdated = ahtValid
      ? climateSensor.setTemperature(temperature)
      : climateSensor.setTemperature(24.0f);

    const bool humidityUpdated = ahtValid
      ? climateSensor.setHumidity(humidity)
      : climateSensor.setHumidity(50.0f);

    const bool co2Updated = co2Valid
      ? carbonDioxideSensor.setCarbonDioxide(co2Value)
      : carbonDioxideSensor.setCarbonDioxide(0);

    if (co2Updated && co2Valid) {
      Serial.printf("Temp: %.1f C  Hum: %.1f%%  CO2: %u ppm (+- %.1f%%)\n",
                    temperature, humidity, co2Value, co2Accuracy);
    } else {
      Serial.printf("Temp: %.1f C  Hum: %.1f%%  CO2: N/A\n",
                    temperature, humidity);
    }

    if (!temperatureUpdated || !humidityUpdated || !co2Updated) {
      Serial.println("Failed to update one or more Zigbee sensor values");
    }

    if (!automaticReportingConfigured) {
      if (!climateSensor.report()) {
        Serial.println("Failed to report temperature and humidity");
      }
      if (!carbonDioxideSensor.report()) {
        Serial.println("Failed to report CO2");
      }
    }
  }

  delay(10);
}
