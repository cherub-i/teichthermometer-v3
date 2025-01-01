#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <Wifi.h>

#include "secrets.h"

// Config
#define ONE_WIRE_BUS 4

#ifndef SECRETS_H
// WIFI access
const char* kHostname = "<hostname>";
const char* kWifiSsid = "<wifi-ssid>";
const char* kWifiPassword = "<wifi-pwd>>";

// MQTT access
const char* kMqttServer = "<mqtt.server.com>";
const int kMqttPort = 1883;
const char* kMqttClientId = "<client-id>";
const char* kMqttUser = "<user-name>";
const char* kMqttPassword = "<user-password>";
const char* kMqttTopic = "<topic/subtopic>";
#endif

// Energy saving
const boolean kDevelopmentMode = false;
const int kSleepTimeSeconds = (kDevelopmentMode) ? 10 : 10 * 60;
const int kMaxTimeWithoutTransmissionSeconds =
    (kDevelopmentMode) ? 30 : 60 * 60;
const float kMinTemperatureDifference = 0.1;

// Sensor Byte-Adresses
const int kNumberOfSensors = 2;
const DeviceAddress kSensorAdresses[] = {
    {0x28, 0x36, 0x56, 0xC4, 0x17, 0x20, 0x6, 0x53},
    {0x28, 0xBF, 0xE0, 0xCD, 0x17, 0x20, 0x6, 0x6F}};

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int transmissionsSkippedCount = 0;
RTC_DATA_ATTR float previousTransmittedTemperatures[kNumberOfSensors];

// globals
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
StaticJsonDocument<96> jsonOut;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void read_sensor_data(float* temperatures) {
    sensors.begin();
    sensors.requestTemperatures();
    for (int i = 0; i < kNumberOfSensors; i++) {
        temperatures[i] = sensors.getTempC(kSensorAdresses[i]);
    }
}

bool is_relevant_change(float temperature_1, float temperature_2) {
    return abs(temperature_1 - temperature_2) > kMinTemperatureDifference;
}

double round2(double value) { return (int)(value * 100 + 0.5) / 100.0; }

void setup_serial_and_log() {
    Serial.begin(9600);
    while (!Serial && !Serial.available()) {
    }
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
}

void setup_wifi() {
    Log.noticeln("WiFi: connecting to %s ...", kWifiSsid);
    WiFi.disconnect();
    WiFi.setHostname(kHostname);
    WiFi.begin(kWifiSsid, kWifiPassword);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
    }
    Log.noticeln("... SUCCESS");
}

void setup_mqtt() {
    Log.noticeln("MQTT: connecting to %s as user %s with id %s ...",
                 kMqttServer, kMqttUser, kMqttClientId);
    mqttClient.setServer(kMqttServer, kMqttPort);
    mqttClient.connect(kMqttClientId, kMqttUser, kMqttPassword);
    if (mqttClient.connected()) {
        Log.noticeln("... SUCCESS");
    } else {
        Log.errorln("MQTT Connection failed");
        Log.errorln("MQTT client state: %d", mqttClient.state());
    }
}

void deep_sleep(int sleepTimeSeconds) {
    Log.noticeln("Setting timer to %ds", sleepTimeSeconds);
    esp_sleep_enable_timer_wakeup(sleepTimeSeconds * 1000000);

    Log.noticeln("Starting deep sleep - Byebye ###");
    Serial.flush();
    esp_deep_sleep_start();
}

void setup() {
    ++bootCount;
    boolean transmissionNeeded = false;
    char mqttMessage[256];

    setup_serial_and_log();

    Log.noticeln("### Starting");
    Log.noticeln("Boot count: %d, Transmissions skipped count: %d", bootCount,
                 transmissionsSkippedCount);

    // Read sensors
    float temperatures[kNumberOfSensors];
    String changeType;
    read_sensor_data(&temperatures[0]);
    for (int i = 0; i < kNumberOfSensors; i++) {
        if (is_relevant_change(temperatures[i],
                               previousTransmittedTemperatures[i])) {
            transmissionNeeded = true;
            changeType = "relevant";
        } else {
            changeType = "insignificant";
        }
        Log.noticeln("Sensor %d: %F (previously transmitted: %F): %s change", i,
                     temperatures[i], previousTransmittedTemperatures[i],
                     changeType);
    }

    // Check time without transmissions
    int timeWithoutTransmissionSeconds =
        (transmissionsSkippedCount * kSleepTimeSeconds) + kSleepTimeSeconds;
    Log.noticeln("Time without transmission: %ds",
                 timeWithoutTransmissionSeconds);
    if (timeWithoutTransmissionSeconds >= kMaxTimeWithoutTransmissionSeconds) {
        Log.noticeln("Last transmission too long ago.");
        transmissionNeeded = true;
    }

    if (transmissionNeeded) {
        setup_wifi();
        setup_mqtt();

        jsonOut["RSSI"] = WiFi.RSSI();
        if (transmissionsSkippedCount > 0) {
            jsonOut["Skipped"] = transmissionsSkippedCount;
        }

        for (int i = 0; i < kNumberOfSensors; i++) {
            jsonOut["Temp_" + String(i)] = round2(temperatures[i]);
            previousTransmittedTemperatures[i] = temperatures[i];
        }

        size_t n = serializeJson(jsonOut, mqttMessage);
        mqttClient.publish(kMqttTopic, mqttMessage, n);
        Log.noticeln("Sent MQTT Message: %s", mqttMessage);
        transmissionsSkippedCount = 0;
    } else {
        Log.noticeln("No transmission needed");
        ++transmissionsSkippedCount;
    }

    deep_sleep(kSleepTimeSeconds);
}

void loop() {
    // put your main code here, to run repeatedly:
}
