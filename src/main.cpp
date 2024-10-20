#include <Arduino.h>
#include <WifiConfig.hpp>
#include <Wire.h>
#include <SHT2x.h>
#include <Adafruit_SH110X.h>
#include <arduino-timer.h>
#include <OneButton.h>
#include <time.h>
#include <ESP32Servo.h>
#include <HAswitchHelper.hpp>
#include <HAnumberHelper.hpp>
#include <HAfanHelper.hpp>
#include <SerialHandler.hpp>
#include "defines.hpp"
#include "secrets.h"

void updateDisplay();
void setupPeripherals();
void readSensor();
void serialCb(const String&);
void servoPosCb(int);

bool debug = true;
WifiConfig wifiConfig(WIFI_SSID, WIFI_PASSWORD, "ESP32 Thermo-Clock Hub", "thermo-clock", AUTH_USER, AUTH_PASS, true, true, debug);
SHT2x sensor;
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Servo servo;
Timer<4> timer;

bool hasSensor = false;
float temp = 0.0;
float humi = 0.0;
IntConfig PIR("PIR", 0);
IntConfig rawPIR("rawPIR", 0);
void *pirDelay = nullptr;
DisplayOffsets d_offsets;
HAswitchHelper servo_sw(wifiConfig, "servo_sw", SERVO_SW_PIN, false, debug);
HAnumberHelper servo_pos(wifiConfig, "servo_pos", servoPosCb, 90, 0, 180, 1, debug);
HAfanHelper fan_1(wifiConfig, "fan_1", FAN_PIN, 8, 0, 0, false, debug);
HAfanHelper fan_2(wifiConfig, "fan_2", FAN2_PIN, 8, 0, 0, false, debug);

void setup() {
  if (debug) {
    Serial.begin(115200);
    delay(10);
  }

  setupPeripherals();
  randomSeed(analogRead(RANDOM_SEED_PIN));
  configTzTime(CLOCK_TIMEZONE, NTP_SERVER);

  rawPIR.setCb([](int value) {
    if (value == HIGH) {
      if (pirDelay) timer.cancel(pirDelay);
      PIR.setValue(HIGH);
    } else {
      pirDelay = timer.in(30000, [](void*) -> bool {
        PIR.setValue(LOW);
        pirDelay = nullptr;
        return true;
      });
    }
  });
  PIR.setCb([](int value) {
    wifiConfig.publish("binary_sensor/{sensorId}_PIR/state", value ? "ON" : "OFF", true);
  });

  wifiConfig.beginMQTT(
    MQTT_SERVER,
    1883,
    MQTT_USER,
    MQTT_PASS,
    "homeassistant/",
    MQTTConnectProps([]() {
      wifiConfig.publish("binary_sensor/{sensorId}_PIR/config", wifiConfig.binarySensorConfigPayload("PIR", "motion"), true);
      if (hasSensor) {
        wifiConfig.publish("sensor/{sensorId}_temperature/config", wifiConfig.sensorConfigPayload("temperature", "temperature", "°C"), true);
        wifiConfig.publish("sensor/{sensorId}_humidity/config", wifiConfig.sensorConfigPayload("humidity", "humidity", "%"), true);
        readSensor();
      }
      servo_sw.onMqttConnect();
      servo_pos.onMqttConnect();
      fan_1.onMqttConnect();
      fan_2.onMqttConnect();
    }, [](String topic, String data) {
      servo_sw.onMqttMessage(topic, data);
      servo_pos.onMqttMessage(topic, data);
      fan_1.onMqttMessage(topic, data);
      fan_2.onMqttMessage(topic, data);
    })
  );

  servo_sw.begin();
  servo_pos.begin();
  fan_1.begin();
  fan_2.begin();
  servo.attach(SERVO_PIN);

  display.setRotation(2);
  display.display();

  timer.every(1000, [](void*) -> bool { updateDisplay(); return true; });
  timer.every(10000, [](void*) -> bool { d_offsets.randomize(); return true; });
  if (hasSensor) {
    timer.every(60000, [](void*) -> bool { readSensor(); return true; });
  }
}

void loop() {
  rawPIR.setValue(digitalRead(PIR_PIN));
  wifiConfig.loop();
  timer.tick();
  fan_1.tick();
  fan_2.tick();
  handleSerial(debug, serialCb);
}

void updateDisplay() {
  struct tm timeinfo;
  display.clearDisplay();
  if (PIR.getIntVal() == 0) {
    display.display();
    return;
  }

  if (wifiConfig.isWifiConnected()) getLocalTime(&timeinfo);

  // display temperature and humidity on top line
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0 + d_offsets.line_1_x, 0);
  display.print(temp, 1);
  display.print("C ");
  display.print(humi, 1);
  display.println("%");

  display.setCursor(0 + d_offsets.clock_x, 12 + d_offsets.clock_y);
  display.setTextSize(2);
  display.println(&timeinfo, "%H:%M:%S");

  display.display();
}

void setupPeripherals() {
  pinMode(RANDOM_SEED_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  Wire.begin();
  hasSensor = sensor.begin(&Wire);
  if (debug) {
    uint8_t stat = sensor.getStatus();
    Serial.print("Sensor status: ");
    Serial.print(stat, HEX);
    Serial.println();
  }
  if (display.begin(SCREEN_ADDRESS, true)) {
    if (debug) Serial.println(F("SH110X started"));
  }
}

void readSensor() {
  if (debug) Serial.println("Reading sensor");

  sensor.read();
  temp = sensor.getTemperature();
  humi = sensor.getHumidity();
  wifiConfig.publish("sensor/{sensorId}_temperature/state", String(temp), true);
  wifiConfig.publish("sensor/{sensorId}_humidity/state", String(humi), true);
}

void serialCb(const String& data) {
  // do nothing atm
}

void servoPosCb(int value) {
  servo.write(value);
}
