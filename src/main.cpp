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
#define SEALEVELPRESSURE_HPA (1013.25)

float temperature, humidity, pressure, altitude;

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

TwoWire I2Cone = TwoWire(1);
Adafruit_BME280 bme;

void setup() {
    Serial.begin(115200);
    Serial.println("started...");

    // setup the BME280 sensor
    I2Cone.begin(SDA, SCL, 100000); 
    bool status_bme = bme.begin(0x76, &I2Cone);  
    if (!status_bme) {
        Serial.println("Could not find a valid BME280_1 sensor, check wiring!");
        while (1); // effectively halts the whole system
    }

    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();
  
    display.setFont (ArialMT_Plain_16);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (0, 0, "IoT für eine bes-");
    display.drawString (0, 20, "sere (Um-)Welt");
    display.setFont (ArialMT_Plain_10);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (0, 50, "       ...mit T-Systems MMS");
    display.display ();
}

void loop() {
    // read data
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
    // log data
    Serial.println("temperature = " + String(temperature));
    Serial.println("humidity = " + String(humidity));
    Serial.println("pressure = " + String(pressure));
    Serial.println("altitude = " + String(altitude));

    sleep(5);
}
