// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <Adafruit_BME280.h>

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define SDA 21
#define SCL 13

#define FIXED_HEIGHT 270      // meters
#define FIXED_PRESSURE 990.61 // hPa

#define DISPLAY_UPDATE_RATE 5 // seconds

float seaLevelPressure;
float temperature, humidity, pressure, altitude;

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

enum DisplayState
{
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

    // setup the BME280 sensor
    I2Cone.begin(SDA, SCL, 100000);
    bool status_bme = bme.begin(0x76, &I2Cone);
    if (!status_bme)
    {
        Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
        while (1)
            ; // effectively halts the whole system
    }

    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();

    display.setTextAlignment(TEXT_ALIGN_LEFT);

    // calculate current sea level pressure
    seaLevelPressure = calcSeaLevelPressure(FIXED_HEIGHT /*m*/, FIXED_PRESSURE /*hPa*/);
    Serial.println("seaLevelPressure = " + String(seaLevelPressure) + "hPa");
    Serial.println("============================");
}

void loop()
{
    // read & log data
    display.resetDisplay();

    switch (displayState)
    {
    case TEMPERATURE:
        temperature = bme.readTemperature();
        Serial.println("temperature = " + String(temperature) + "°C");
        drawMeasurement("Temperature:", String(temperature) + "°C");
        displayState = HUMIDITY;
        break;
    case HUMIDITY:
        humidity = bme.readHumidity();
        Serial.println("humidity    = " + String(humidity) + "%");
        drawMeasurement("Humiditiy:", String(humidity) + "%");
        displayState = PRESSURE;
        break;
    case PRESSURE:
        pressure = bme.readPressure() / 100.0F;
        Serial.println("pressure    = " + String(pressure) + "hPa");
        drawMeasurement("Pressure:", String(pressure) + "hPa");
        displayState = ALTITUDE;
        break;
    case ALTITUDE:
        altitude = bme.readAltitude(seaLevelPressure);
        Serial.println("altitude    = " + String(altitude) + "NHN");
        drawMeasurement("Altitude:", String(altitude) + "NHN");
        displayState = TEMPERATURE;
        Serial.println("=======================");
        break;
    default:
        Serial.println("No display action for " + String(displayState) + " found.");
        break;
    }

    display.display();

    sleep(DISPLAY_UPDATE_RATE);
}