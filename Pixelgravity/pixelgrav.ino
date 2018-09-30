/*

  HelloWorld.ino

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <HardwareTimer.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <stdio.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define PERIODuS 7000000U

#define MATR_X 128   // число светодиодов по х
#define MATR_Y 32    // число светодиодов по у
#define PIXELSIZE 10 // размер пикселя мм

// ФИЗИКА
#define G_CONST 0.7 //9.81 // ускорение свободного падения
#define FRICTION 1  // трение
#define MIN_STEP 30 // минимальный шаг интегрирования (миллисекункды) def: 30
// при сильном уменьшении шага всё идёт по п*зде, что очень странно для Эйлера...

// ЭФФЕКТЫ
#define PIXEL_AMOUNT 300 // число "живых" пикселей

#define MATR_X_M MATR_X*PIXELSIZE // размер матрицы в миллиметрах х
#define MATR_Y_M MATR_Y*PIXELSIZE // размер матрицы в миллиметрах у
#define oyMAX 10000
#define oyMIN 9000

int x_vel[PIXEL_AMOUNT];  // МИЛЛИМЕТРЫ В СЕКУНДУ (мм/сек)
int y_vel[PIXEL_AMOUNT];  // МИЛЛИМЕТРЫ В СЕКУНДУ
int x_dist[PIXEL_AMOUNT]; // МИЛЛИМЕТРЫ (мм)
int y_dist[PIXEL_AMOUNT]; // МИЛЛИМЕТРЫ
uint16_t friction[PIXEL_AMOUNT];
uint16_t bounce[PIXEL_AMOUNT];

float mpuPitch;
float mpuRoll;
int16_t ax = 20000, ay = 3000; //Эмуляция получения значений с аккселерометра
//int16_t gx, gy, gz;

unsigned long integrTimer, loopTimer;
float stepTime;
uint8_t varr = 0;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

void handleIntTimer3(void)
{ // Инвертируем переменные для симуляции поворота
  varr ^= 1;
  ax *= (-1);
  ay *= (-1);
}

void changeAngle()
{ ////Функция для изменения значения по оси OY туда-сюда 3000..7000

  static uint8_t oldVarr = 0, dnFallFlag = 0;
  if (ay > 0)
  {
    if (oldVarr != varr)
    {
      if ((ay <= oyMAX) && dnFallFlag == 0)
      {
        ay += 500;
        Serial.print("\\n");
        Serial.print(ay);

      }
      if (ay > oyMAX)
        dnFallFlag = 1;

      if ((ay >= oyMIN) && dnFallFlag)
      {
        ay -= 500;
        Serial.print("\\n");
        Serial.print(ay);
      }
      if (ay < oyMIN)
        dnFallFlag = 0;
    }
    //Serial.print("\nay = ");
    //Serial.print(ay);
    oldVarr = varr;
  }
}

void rotateStix(void)
{
  for (uint16_t i = 0; i < 10; i++)
  {
    u8g2.clearBuffer();
    u8g2.drawStr(10, 20, "Kim Czulik |"); // write something to the internal memory
    u8g2.drawStr(50, 30, "2018 |");
    u8g2.sendBuffer();
    delay(100);
    u8g2.clearBuffer();
    u8g2.drawStr(10, 20, "Kim Czulik /"); // write something to the internal memory
    u8g2.drawStr(50, 30, "2018 \\");
    u8g2.sendBuffer();
    delay(100);
    u8g2.clearBuffer();
    u8g2.drawStr(10, 20, "Kim Czulik --"); // write something to the internal memory
    u8g2.drawStr(50, 30, "2018 --");
    u8g2.sendBuffer();
    delay(100);
    u8g2.clearBuffer();
    u8g2.drawStr(10, 20, "Kim Czulik \\"); // write something to the internal memory
    u8g2.drawStr(50, 30, "2018 /");
    u8g2.sendBuffer();
    delay(100);
  }
}

void setup(void)
{
  pinMode(0, INPUT_ANALOG);
  Serial.begin(9600);
  u8g2.begin();
  u8g2.clearBuffer();                 // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  //u8g2.drawStr(10, 20, "Kim Czulik"); // write something to the internal memory
  //u8g2.sendBuffer();
  //delay(1000);
  rotateStix();

  u8g2.clearBuffer();

  Timer3.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);    //Настройка таймера(канал, режим)
  Timer3.pause();                                     //Останов таймера для его настройки
  Timer3.setPeriod(PERIODuS);                         //Установка периода одного тика таймера
                                                      // Timer3.setCompare(TIMER_CH1, 5U); //Установка значения сравнения количества тиков для вызова прерывания
  Timer3.attachInterrupt(TIMER_CH1, handleIntTimer3); //Вызов прерывания с передачей значения по ссылке
  Timer3.resume();

  ///////Нулевые условия. Центр матрицы
  for (uint16_t i = 0; i < PIXEL_AMOUNT; i++)
  {
    x_vel[i] = 0;
    y_vel[i] = 0;
    x_dist[i] = (MATR_X_M / 2);
    y_dist[i] = (MATR_Y_M / 2);
  }
  randomSeed(analogRead(0));

  for (uint16_t i = 0; i < PIXEL_AMOUNT; i++)
  {
    // получаем случайные величины
    friction[i] = random(0, 30); // ТРЕНИЕ. В дальнейшем делится на 100
    bounce[i] = random(60, 80);  // ОТСКОК. В дальнейшем делится на 100  дефолт: (60, 95)
  }
}

void loop(void)
{

  //Serial.print(Timer3.getPrescaleFactor());
  //char buff[10];   //Массив символов для конвертирования целочисленного типа в строку
  //sprintf(buff, "%d", varr); //Преобразовываем int в string
  //u8g2.drawStr(0, 20, buff);

  if ((millis() - loopTimer) > MIN_STEP)
  {
    loopTimer = millis();
    integrate();
    u8g2.clearBuffer();
    for (uint16_t i = 0; i < PIXEL_AMOUNT; i++)
    {
      uint16_t nowDistX = round(x_dist[i] / PIXELSIZE); // перевести миллиметры в пиксели floor
      uint16_t nowDistY = round(y_dist[i] / PIXELSIZE);
      u8g2.drawPixel(nowDistX, nowDistY);
    }
    u8g2.sendBuffer();
  }
  changeAngle();
}

void integrate()
{
  mpuPitch = (float)ax / 16384; // 16384 это величина g с акселерометра
  mpuRoll = (float)ay / 16384;

  stepTime = (float)((long)millis() - integrTimer) / 1000; // расчёт времени шага интегрирования
  integrTimer = millis();
  for (uint16_t i = 0; i < PIXEL_AMOUNT; i++)
  { // для каждого пикселя
    int thisAccel_x, thisAccel_y;
    int grav, frict;

    ///////////////////// ОСЬ Х /////////////////////
    grav = (float)G_CONST * mpuPitch * 1000; // сила тяжести
    if (FRICTION)
    {
      frict = (float)G_CONST * (1 - mpuPitch) * friction[i] * 10; // сила трения
      if (x_vel[i] > 0)
        frict = -frict; // знак силы трения зависит от направления вектора скорости
      if (x_vel[i] == 0 && abs(grav) < frict)
        thisAccel_x = 0; // трение покоя
      else
        thisAccel_x = (grav + frict); // ускорение
    }
    else
      thisAccel_x = grav;

    /////////////////////// ОСЬ У /////////////////////
    grav = (float)G_CONST * mpuRoll * 1000;
    if (FRICTION)
    {
      frict = (float)G_CONST * (1 - mpuRoll) * friction[i] * 10;
      if (y_vel[i] > 0)
        frict = -frict;
      if (y_vel[i] == 0 && abs(grav) < frict)
        thisAccel_y = 0;
      else
        thisAccel_y = (grav + frict);
    }
    else
      thisAccel_y = grav;

    ///////////////////// ИНТЕГРИРУЕМ ///////////////////
    // скорость на данном шаге V = V0 + ax*dt
    x_vel[i] += (float)thisAccel_x * stepTime;
    y_vel[i] += (float)thisAccel_y * stepTime;

    // координата на данном шаге X = X0 + Vx*dt
    x_dist[i] += (float)x_vel[i] * stepTime;
    y_dist[i] += (float)y_vel[i] * stepTime;

    /////////////////// ПОВЕДЕНИЕ У СТЕНОК /////////////////
    // рассматриваем 4 стенки матрицы
    if (x_dist[i] < 0)
    {                                                // если пробили край матрицы
      x_dist[i] = 0;                                 // возвращаем на край
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100; // скорость принимаем с обратным знаком и * на коэффициент отскока
    }
    if (x_dist[i] > MATR_X_M - PIXELSIZE)
    {
      x_dist[i] = MATR_X_M - PIXELSIZE;
      x_vel[i] = -x_vel[i] * (float)bounce[i] / 100;
    }

    if (y_dist[i] < 0)
    {
      y_dist[i] = 0;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
    if (y_dist[i] > MATR_Y_M - PIXELSIZE)
    {
      y_dist[i] = MATR_Y_M - PIXELSIZE;
      y_vel[i] = -y_vel[i] * (float)bounce[i] / 100;
    }
  }
}
