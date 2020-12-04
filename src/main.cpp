// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <Arduino.h>
#include <SPI.h>
#include <SSD1306.h>

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

unsigned int counter = 0;

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

void setup() {
    pinMode(LEDPIN, OUTPUT);

    Serial.begin(115200);
    Serial.println("started...");

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
    Serial.println(counter++);
    digitalWrite(LEDPIN, HIGH);
    sleep(1);
    digitalWrite(LEDPIN, LOW);
    sleep(1);
}
