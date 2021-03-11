// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>

#include <ctime>

#define OLED_I2C_ADDR 0x3c
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define SDA 21
#define SCL 13

#define FIXED_HEIGHT 270      // meters
#define FIXED_PRESSURE 990.61 // hPa

#define DISPLAY_UPDATE_RATE 5 // seconds

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

float seaLevelPressure;
float temperature, humidity, pressure, altitude;

time_t now;
const int arraySize = 155;
char timestamp;
tm nowlocal;

//SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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


void drawSensorInformations(String title, String content, int title_x, int content_x)
{
    display.clearDisplay();

    display.setCursor(title_x, 15);
    display.print(title);

    display.setCursor(content_x, 48);
    display.print(content);

    //x, y, Breite, Höhe, Radius der Ecken, Farbe
    display.drawRoundRect(0, 26, 127, 35, 14, WHITE);
    Serial.println("drawed");
}

void drawTime(int hours, int minutes){

    display.clearDisplay();
    display.setFont(&FreeMonoBold18pt7b);
    display.setCursor(10, 46);
                    
    //only for now; later query via wifi
    //local time starts at 00:00 01.01.1970
    time_t now = time(nullptr);
    char timestamp[arraySize];
    tm nowlocal = *localtime(&now);
    strftime(timestamp, arraySize - 1, "%H:%M", &nowlocal);

    Serial.println(timestamp);
    display.print(timestamp);

    display.drawRoundRect(6, 8, 116, 56, 14, WHITE);
    display.setFont(&FreeMonoBold9pt7b);

}

void setup()
{
    Serial.begin(115200);
    Serial.println("started...");

    Wire.begin(4, 15); display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
    delay(1000);

    //setup the BME280 sensor
    I2Cone.begin(SDA, SCL, 100000);
    bool status_bme = bme.begin(0x76, &I2Cone);
    if (!status_bme)
    {
        Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
        while (1)
            ; // effectively halts the whole system
    }


    display.clearDisplay();
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(WHITE);
    display.setCursor(0, 15);
    display.display();

    // calculate current sea level pressure 
    seaLevelPressure = calcSeaLevelPressure(FIXED_HEIGHT /*m*/, FIXED_PRESSURE /*hPa*/);
    Serial.println("seaLevelPressure = " + String(seaLevelPressure) + "hPa");
    Serial.println("============================"); 
}

void loop()
{

    switch (displayState)
    {
    case TEMPERATURE:
        temperature = bme.readTemperature();
        Serial.println("temperature = " + String(temperature) + "°C");
        drawSensorInformations("Temperature:", String(temperature) + "°C", 0, 30);
        displayState = HUMIDITY;

        break;
    case HUMIDITY:
        humidity = bme.readHumidity();
        Serial.println("humidity    = " + String(humidity) + "%");
        drawSensorInformations("Humidity:", String(humidity) + "%", 16, 32);
        displayState = PRESSURE;

        break;
    case PRESSURE:
        pressure = bme.readPressure() / 100.0F;
        Serial.println("pressure    = " + String(pressure) + "hPa");
        drawSensorInformations("Pressure:", String(pressure) + "hPa", 15, 8);
        displayState = ALTITUDE;

        break;
    case ALTITUDE:
        altitude = bme.readAltitude(seaLevelPressure);
        Serial.println("altitude    = " + String(altitude) + "NHN");
        drawSensorInformations("Altitude:", String(altitude) + "NHN", 15, 12);
        displayState = TEMPERATURE;

        Serial.println("=======================");
        break;
    default:
        Serial.println("No display action for " + String(displayState) + " found.");
        break;
    }

    display.display();
    sleep(DISPLAY_UPDATE_RATE);

    drawTime(06, 02);
    display.display();
    sleep(DISPLAY_UPDATE_RATE);

}