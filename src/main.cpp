// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <Adafruit_BME280.h>
#include <wifi.h>
#include <NTPClient.h>
#include <PubSubClient.h>

#define SSID "WLAN Marcel"
#define SSID_PASS "Das ist ein sicheres Passwort11!!!1"

#define MQTT_ADDRESS "192.168.178.29"
#define MQTT_PORT 1883

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define SDA 21
#define SCL 13

#define FIXED_HEIGHT 270      // meters
#define FIXED_PRESSURE 990.61 // hPa

#define DISPLAY_UPDATE_RATE 10 // seconds

float seaLevelPressure;
float temperature, humidity, pressure, altitude;

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

Wifi wifi(SSID, SSID_PASS);
WiFiUDP udp;
NTPClient timeClient(udp, "de.pool.ntp.org");

WiFiClient client;
PubSubClient mqttClient(client);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

enum DisplayState
{
    TIME,
    TEMPERATURE,
    HUMIDITY,
    PRESSURE,
    ALTITUDE
};

DisplayState displayState = TEMPERATURE;

float calcSeaLevelPressure(float height, float pressure)
{
    return pressure / pow(1 - (height / 44330), 1 / 0.1903);
}

void drawMeasurement(String title, String value)
{
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, title);
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 20, value);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("started...");

    Serial.print("Setting up bme280...");
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
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    Serial.println(" done.");

    Serial.print("Setting up mqtt...");
    mqttClient.setServer(MQTT_ADDRESS, MQTT_PORT);
    mqttClient.setKeepAlive(DISPLAY_UPDATE_RATE + 5);
    mqttClient.setSocketTimeout(DISPLAY_UPDATE_RATE + 5);
    Serial.println(" done.");

    // calculate current sea level pressure
    seaLevelPressure = calcSeaLevelPressure(FIXED_HEIGHT /*m*/, FIXED_PRESSURE /*hPa*/);
    Serial.println("seaLevelPressure = " + String(seaLevelPressure) + "hPa");
    Serial.println("============================");
}

void loop()
{
    if (displayState != TIME)
    {
        // check wifi
        if (!wifi.update())
        {
            return;
        }

        // check mqtt
        if (!mqttClient.connected())
        {
            Serial.print("Connection to mqtt...");
            if (mqttClient.connect("T-Systems MMS Arduindo (BME)"))
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
    }

    // read & log data
    display.resetDisplay();

    switch (displayState)
    {
    case TIME:
        Serial.println("=======================");

        // update time
        timeClient.update();

        Serial.println("time        = " + timeClient.getFormattedTime());
        drawMeasurement("Time:", timeClient.getFormattedTime());
        displayState = TEMPERATURE;
        break;
    case TEMPERATURE:
        temperature = bme.readTemperature();
        Serial.println("temperature = " + String(temperature) + "°C");
        drawMeasurement("Temperature:", String(temperature) + "°C");
        mqttClient.publish("sensor_id/188628af-3461-4e34-affc-2841a6e89fd9", String(temperature).c_str());
        displayState = HUMIDITY;
        break;
    case HUMIDITY:
        humidity = bme.readHumidity();
        Serial.println("humidity    = " + String(humidity) + "%");
        drawMeasurement("Humiditiy:", String(humidity) + "%");
        mqttClient.publish("sensor_id/fdbcac5f-48a4-48f4-9449-9a87f111bf2c", String(humidity).c_str());
        displayState = PRESSURE;
        break;
    case PRESSURE:
        pressure = bme.readPressure() / 100.0F;
        Serial.println("pressure    = " + String(pressure) + "hPa");
        drawMeasurement("Pressure:", String(pressure) + "hPa");
        mqttClient.publish("sensor_id/0df22e27-a8ca-4c18-b1a4-575502f40b61", String(pressure).c_str());
        displayState = TIME; //ALTITUDE
        break;
    case ALTITUDE:
        altitude = bme.readAltitude(seaLevelPressure);
        Serial.println("altitude    = " + String(altitude) + "NHN");
        drawMeasurement("Altitude:", String(altitude) + "NHN");
        displayState = TIME;
        break;
    default:
        Serial.println("No display action for " + String(displayState) + " found.");
        break;
    }

    display.display();

    mqttClient.loop();
    sleep(DISPLAY_UPDATE_RATE);
}