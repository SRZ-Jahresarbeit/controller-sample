// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <Adafruit_BME280.h>
#include <classWifi.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <ctime>
#include <PubSubClient.h>

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define DISPLAY_UPDATE_RATE 5 // seconds

#define SSID "WLAN Marcel"
#define SSID_PASS "Das ist ein sicheres Passwort11!!!1"

#define MQTT_ADDRESS "192.168.178.39"
#define MQTT_PORT 1883

#define SDA 21
#define SCL 13
//#define SEALEVELPRESSURE_HPA (1013.25)

float temperature, humidity, pressure /*, altitude*/;

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

Wifi wifi(SSID, SSID_PASS);
WiFiUDP udp;
NTPClient timeClient(udp, "de.pool.ntp.org");

WiFiClient client;
PubSubClient mqttClient(client);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

void setup()
{
    Serial.begin(115200);
    sleep(5); // wait for serial

    Serial.println("started...");

    Serial.print("Setting up bme280...");
    // setup the BME280 sensor
    I2Cone.begin(SDA, SCL, 100000);
    bool status_bme = bme.begin(0x76, &I2Cone);
    if (!status_bme)
    {
        Serial.println(" error.");
        Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
        while (1)
            ; // effectively halts the whole system
    }
    Serial.println(" done.");

    Serial.print("Setting up display...");
    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();

    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    Serial.println(" done.");

    Serial.print("Setting up mqtt...");
    mqttClient.setServer(MQTT_ADDRESS, MQTT_PORT);
    Serial.println(" done.");
}

void loop()
{
    if (!wifi.update())
    {
        return;
    }

    if (!mqttClient.connected())
    {
        Serial.print("Connection to mqtt...");
        if (mqttClient.connect("T-Systems Arduino 001"))
        {
            Serial.println(" done.");
        }
        else
        {
            Serial.println(" error.");
            sleep(10); // seconds
            return;
        }
    }

    // update current time (may don't do this in every tick)
    timeClient.update();

    // read data from bme280
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    // altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // publish data
    mqttClient.publish("temperature/001", String(temperature).c_str());
    mqttClient.publish("humidity/001", String(humidity).c_str());
    mqttClient.publish("pressure/001", String(pressure).c_str());
    // mqttClient.publish("altitude/001", String(altitude).c_str());

    // publish current time
    mqttClient.publish("time/001", String(timeClient.getEpochTime()).c_str());

    // read & log data
    display.resetDisplay();

    display.drawString(0, 0, WiFi.getHostname());
    display.drawString(0, 20, timeClient.getFormattedTime());

    display.display();

    mqttClient.loop();

    sleep(DISPLAY_UPDATE_RATE);
}