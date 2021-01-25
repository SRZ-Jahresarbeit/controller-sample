// MIT License
// Copyright (c) 2021 Marco Grunert

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>
#include <wifi.h>
#include <NTPClient.h>
#include <ctime>

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define DISPLAY_UPDATE_RATE 5 // seconds

#define SSID "Handy Marcel"
#define SSID_PASS "Das ist ein sicheres Passwort!"

SSD1306 display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

Wifi wifi(SSID, SSID_PASS);
WiFiUDP udp;
NTPClient timeClient(udp, "de.pool.ntp.org");

void setup()
{
    Serial.begin(115200);
    sleep(5); // wait for serial

    Serial.println("started...");

    Serial.print("Setting up display... ");
    //Set up and reset the OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);
    display.init();

    display.setFont(ArialMT_Plain_24);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    Serial.println(" done.");
}

void loop()
{
    if (!wifi.update())
    {
        return;
    }

    timeClient.update();
    // read & log data
    display.resetDisplay();

    display.drawString(0, 0, WiFi.getHostname());
    display.drawString(0, 20, timeClient.getFormattedTime());

    display.display();
    sleep(DISPLAY_UPDATE_RATE);
}