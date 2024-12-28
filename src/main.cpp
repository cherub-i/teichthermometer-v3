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

// Energy saving
const boolean developmentMode = false;
const int sleepTimeSeconds = (developmentMode) ? 10 : 10 * 60;
const float minTemperatureDifference = 0.1;

// Sensor Byte-Adresses
const int numberOfSensors = 2;
const DeviceAddress sensorAdresses[] = {
    {0x28, 0x36, 0x56, 0xC4, 0x17, 0x20, 0x6, 0x53},
    {0x28, 0xBF, 0xE0, 0xCD, 0x17, 0x20, 0x6, 0x6F}};

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int messagesSkippedCount = 0;
RTC_DATA_ATTR float previousTemperatures[numberOfSensors];

// globals
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
StaticJsonDocument<96> jsonOut;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void read_sensor_data(float* temperatures) {
    sensors.begin();
    sensors.requestTemperatures();
    for (int i = 0; i < numberOfSensors; i++) {
        temperatures[i] = sensors.getTempC(sensorAdresses[i]);
    }
}

bool is_relevant_change(float temperature_1, float temperature_2) {
    return abs(temperature_1 - temperature_2) > minTemperatureDifference;
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
    Log.noticeln("Setting timer to %d seconds", sleepTimeSeconds);
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
    Log.noticeln("Boot count: %d", bootCount);

    // Sensors
    float temperatures[numberOfSensors];
    read_sensor_data(&temperatures[0]);
    for (int i = 0; i < numberOfSensors; i++) {
        String changeType = "insignificant";
        if (is_relevant_change(temperatures[i], previousTemperatures[i])) {
            previousTemperatures[i] = temperatures[i];
            transmissionNeeded = true;
            changeType = "relevant";
        }
        Log.noticeln("Sensor %d: %F (previously: %F): %s change", i,
                     temperatures[i], previousTemperatures[i], changeType);
        jsonOut["Temp_" + String(i)] = round2(temperatures[i]);
    }

    if (transmissionNeeded) {
        setup_wifi();
        setup_mqtt();

        jsonOut["RSSI"] = WiFi.RSSI();
        if (messagesSkippedCount > 0) {
            jsonOut["Skipped"] = messagesSkippedCount;
        }

        size_t n = serializeJson(jsonOut, mqttMessage);
        mqttClient.publish(kMqttTopic, mqttMessage, n);
        messagesSkippedCount = 0;
        Log.noticeln("Sent MQTT Message: %s", mqttMessage);
    } else {
        Log.noticeln("No transmission needed");
        ++messagesSkippedCount;
    }

    deep_sleep(sleepTimeSeconds);
}

void loop() {
    // put your main code here, to run repeatedly:
}
