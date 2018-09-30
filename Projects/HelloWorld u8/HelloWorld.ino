#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>

#define OLED_RESET 4

Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup(void)
{
  Serial.begin(9600);


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println("CAH9| MEXAHNK");
  display.display();
  display.drawLine(0, 10, 30, 10, WHITE);
  display.display();
  delay(1000);
}

void loop(void)
{

  for (uint8_t i = 0; i < display.width(); i++)
  {
    for (uint8_t j = 10; j < 12; j++)
    {
      display.drawLine(0, 10, i, j, WHITE);
      display.display();
      delay(41);
    }
  }
  display.clearDisplay();
}
