// Автоматизация для пивной станции Серёги Сафрона.
// Чулик Ким, 03.2020г.
#define _LCD_TYPE 1 // Определение I2C протокола для LCD
#include <Arduino.h>
#include "GyverButton.h"
#include "GyverRelay.h"
#include "Wire.h"
#include <GyverEncoder.h>
//#include "LiquidCrystal_I2C.h"
#include <LCD_1602_RUS_ALL.h> // вместо I2C библиотеки - в рус.версии уже прописано всё.
#include "LiquidMenu.h"
#include "NTC_Thermistor.h"
#include <GyverFilters.h>

//------------НАСТРОЙКИ------------
//Термосопротивление
#define SENSOR_PIN PB1               // Пин сенсора по схеме: (+)---NTC---(Ain)---Rreference---(-)
#define REFERENCE_RESISTANCE 6780    // Омы опорного резистора
#define NOMINAL_RESISTANCE 10000     // Сопротивление терморезистора
#define NOMINAL_TEMPERATURE 25       // Номинальная температура
#define B_VALUE 3950                 // B-коэффициент default = 3950
#define STM32_ANALOG_RESOLUTION 4095 // Разрешение АЦП

//Реле и сигнал
#define ALARM_PIN PA10
#define SOLID_RELAY_HEATER_PIN PB15 // Пин твердотельного реле на нагреватель
#define RELAY_PUMP_PIN PA8          // Пин реле на помпу
#define RELAY_RESERVED PA9
#define INVERSED_PUMP_LOGIC 1

#define MAX_ZATIR_MODES 6   // Количество ступеней затирки
#define MAX_WARKA_ZABROSY 6 // Количество ступеней варки-закидывания хмеля

//------------Прототипы функций----------
void initStation();
void runStation();
void zatiranie();
void warka();
bool waitForNextZatMode();
void isTimeForNextHmel_Warka();
void userIO();
void initLCDmenu();
void runPumpZatirka(bool enableFlag);
void runPumpWarka(bool enableFlag);
void runPump(bool state, bool inverseFlag);
const char *warkaStatus();
const char *zatirkaStatus();
float returnTemperInCelsius();
void funcSettingsZatirkaMenuOnCheck();
void startZatirkiMenuOnCheck();
void funcSettingsWarkaMenuOnCheck();
void startWarkiMenuOnCheck();
void zatirkaStupienSelectorIncrease();
void zatirkaStupienSelectorDecrease();
uint32_t zatirkaTimeMins();
void zatirkaSetTimeIncrease();
void zatirkaSetTimeDecrease();
void zatirkaTemperSetpointIncrease();
void zatirkaTemperSetpointDecrease();
uint32_t returnNumerStupieniZatirka();
uint32_t zatirkaPumpWorkTime();
uint32_t zatirkaPumpStopTime();
void zatirkaPumpWorkTimeIncrease();
void zatirkaPumpWorkTimeDecrease();
void zatirkaPumpStopTimeIncrease();
void zatirkaPumpStopTimeDecrease();
int32_t returnZatirkaModeTemper();
uint32_t returnSetpointTemperaStupeniZatirka();
uint32_t returnNumerStupeniZatirki();
float timeUntilEndOfZatirka();
void backToMainMenu();
uint32_t warkaTimeMins();
void warkaSetTimeIncrease();
void warkaSetTimeDecrease();
uint32_t returnWarkaTemperature();
void warkaTemperSetpointIncrease();
void warkaTemperSetpointDecrease();
uint32_t returnNumerZabrosaWarka();
void warkaSelectorIncrease();
void warkaModeSelectorIncrease();
void warkaModeSelectorDecrease();
uint32_t warkaStupienTime();
void warkaStupienSetTimeIncrease();
void warkaStupienSetTimeDecrease();
uint32_t warkaPumpWorkTime();
uint32_t warkaPumpStopTime();
void warkaPumpWorkTimeIncrease();
void warkaPumpWorkTimeDecrease();
void warkaPumpStopTimeIncrease();
void warkaPumpStopTimeDecrease();
const char *isPumpRunning();
float timeUntilEndOfStageWarka();
uint32_t timeSinceStartOfModeFunc();
void stopKran();

void ISRenc();
void blankFunction()
{
  return;
}

//-----------Переменные------------

typedef enum
{
  OFF,
  ZATIRANIE,
  WARKA
} stationModes; // 0 - выкл, 1 - затирание, 2 - варка

static bool isFirstRun = 1,
            isZatirkaRunning = 0,
            isWarkaRunning = 0;

stationModes stationCurrentMode = OFF; // User должен выбрать режим работы вручную
static uint32_t timeVarMs,             // Счётчик для логики
    timeSinceStartOfMode;              // Счётчик от начала процесса
static float filteredTemperature = 0; // Переменная в которую положим отфильтрованную температуру
struct Pump
{
  uint32_t timeInWorkMsWarka, // Время работы при варке
      timeInStopMsWarka,      // Время паузы при варке
      timeInWorkMsZatirka,    // Время работы при затирке
      timeInStopMsZatirka;    // Время паузы при затирке
  bool dominantFlag = 1, status;
  const bool runFlag = 1, stopFlag = 0;
} pump;

// ЗАТИРКА
struct zatVars
{
  uint32_t durationTimeMs = 0; // При записи сюда - сконвертировать минуты в миллисекунды
  int32_t temperature = 58;
};
zatVars zatModes[MAX_ZATIR_MODES]{}; // Массив переменных процесса в затирке (дефолтные значения = 0)
static uint32_t numZatMode = 0;      // Номер режима затирки
//-----------

// ВАРКА
static uint32_t numVarZabros = 0; // Номер режима варки
uint32_t warkaDurationTimeMs = 0;
int warkaSetpointTemperature = 100;
uint32_t hmelZabrosTimeMsSinceStart[MAX_WARKA_ZABROSY]{}; // Массив забросов в режиме варки(дефолтные значения = 0)
//-----------

//------------Создание объектов---------
Thermistor *thermistor;
GyverRelay regulator(REVERSE_RELAY);
Encoder encoder(PB13, PB12, PB14, 1);                             //
GKalman filterKalman(14, 0.01);                                   //  Настройка фильтра
LCD_1602_RUS<LiquidCrystal_I2C> lcd(0x27, 20, 4);                 // set the LCD address to 0x27 for a 20 chars and 4 line display
                                                                  //  //-------Добавление строк меню
LiquidLine MainMenuLine1(0, 0, "Ustanovki zatirki");              // Настр.затирки                    //
LiquidLine MainMenuLine2(0, 0, "START zatirki");                  //     СТАРТ Затирки                 //
LiquidLine MainMenuLine3(0, 0, "Ustanovki warki");                //Настройки варки                              //
LiquidLine MainMenuLine4(0, 0, "START warki");                    //СТАРТ Варки                            //
LiquidLine MainMenuLine5(0, 0, "Temper:", returnTemperInCelsius); //ТемпДатчика:  // Тут считает напрямую, функция не нужна

LiquidLine ZatirkaSettingsLine1(0, 0, "-Ustanovki  zatirki-");                   // Настр.затирки
LiquidLine ZatirkaSettingsLine2(0, 0, "Stupien #", returnNumerStupieniZatirka);  //Ступень #
LiquidLine ZatirkaSettingsLine3(0, 0, "Ustan.Time:", zatirkaTimeMins);           // Уст.время:
LiquidLine ZatirkaSettingsLine4(0, 0, "Ustan.temper:", returnZatirkaModeTemper); //Уст.темпер:
LiquidLine ZatirkaSettingsLine5(0, 0, "Pump work time:", zatirkaPumpWorkTime);   //Уст.врем.раб.помпы минут:
LiquidLine ZatirkaSettingsLine6(0, 0, "Pump stop time:", zatirkaPumpStopTime);   //Уст.врем.отдыха помпы минут:
LiquidLine ZatirkaSettingsLine7(0, 0, "BACK");                                   //

LiquidLine WarkaSettingsLine1(0, 0, " -Ustanovki  warki-");                 //Настройки варки
LiquidLine WarkaSettingsLine2(0, 0, "Ust.time warki:", warkaTimeMins);      //Уст.общее время варки:
LiquidLine WarkaSettingsLine3(0, 0, "Ust.temper:", returnWarkaTemperature); //Уст.темпер:
LiquidLine WarkaSettingsLine4(0, 0, "Zabros # ", returnNumerZabrosaWarka);  //Заброс #
LiquidLine WarkaSettingsLine5(0, 0, "Zabros na minute:", warkaStupienTime); //Минут на ступень варки:
LiquidLine WarkaSettingsLine6(0, 0, "Pump work time:", warkaPumpWorkTime);  //Уст.врем.раб.помпы минут:
LiquidLine WarkaSettingsLine7(0, 0, "Pump stop time:", warkaPumpStopTime);  //Уст.врем.отдыха помпы минут:
LiquidLine WarkaSettingsLine8(0, 0, "BACK");

LiquidLine runWarkaLine1(0, 0, warkaStatus);
LiquidLine runWarkaLine2(0, 0, "Numer zabrosa:", returnNumerZabrosaWarka);   // Ном. заброса:
LiquidLine runWarkaLine3(0, 0, "Curr.temper:", returnTemperInCelsius);       // Тек. Темпер =
LiquidLine runWarkaLine4(0, 0, "Do zabrosa:", timeUntilEndOfStageWarka);     // До конца ступени:
LiquidLine runWarkaLine5(0, 0, "Min od zapuska:", timeSinceStartOfModeFunc); //Мин. от запуска
LiquidLine runWarkaLine6(0, 0, "Pump state:", isPumpRunning);
LiquidLine runWarkaLine7(0, 0, "CTOn-KPAH");
LiquidLine runWarkaLine8(0, 0, "BACK");

LiquidLine runZatirkaLine1(0, 0, zatirkaStatus);
LiquidLine runZatirkaLine2(0, 0, "Num.stupeni:", returnNumerStupeniZatirki);           //Ном.ступени:
LiquidLine runZatirkaLine3(0, 0, "Curr.temper:", returnTemperInCelsius);               //Темпер =
LiquidLine runZatirkaLine4(0, 0, "t. Setpoint:", returnSetpointTemperaStupeniZatirka); //До конца:
LiquidLine runZatirkaLine5(0, 0, "Do konca:", timeUntilEndOfZatirka);                  //До конца:
LiquidLine runZatirkaLine6(0, 0, "Pump state:", isPumpRunning);
LiquidLine runZatirkaLine7(0, 0, "CTOn-KPAH");
LiquidLine runZatirkaLine8(0, 0, "BACK");

LiquidScreen screenMainMenu, screenSettingsZatirka, screenSettingsWarka, screenRunZatirka, screenRunWarka;

LiquidMenu mainMenu(lcd);
// LiquidMenu menuSettings(lcd);
// LiquidMenu menuRun(lcd);

//  //
//-------------Encoder прикрутить

void setup() // ############## УСТАНОВКИ ##############
{
  initStation();
}

void loop() // ############## ГЛАВНЫЙ ЦИКЛ ##############
{
  runStation();
}

//****************************************
// ############ ФУНКЦИИ ############
//****************************************

void initStation() //
{
  //****************************************
  // Режимы пинов
  //****************************************
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(RELAY_PUMP_PIN, OUTPUT);
  pinMode(SOLID_RELAY_HEATER_PIN, OUTPUT);
  pinMode(RELAY_RESERVED, OUTPUT);
  Serial.begin(9600);
  //****************************************
  // Останов всех устройств
  //****************************************
  runPumpWarka(pump.stopFlag);
  runPumpZatirka(pump.stopFlag);
  digitalWrite(SOLID_RELAY_HEATER_PIN, 0);
  digitalWrite(RELAY_RESERVED, INVERSED_PUMP_LOGIC);

  // LCD, NTC, Keypad, memoryRead
  encoder.setDirection(NORM);
  // A thermistor instance creating
  thermistor = new NTC_Thermistor(
      SENSOR_PIN,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE,
      STM32_ANALOG_RESOLUTION // <- for a thermistor calibration
  );
  //****************************************
  // НАСТРОЙКА РЕГУЛЯТОРА ГУВЕРА - коэффициенты подбирать!!!!
  //****************************************
  regulator.setpoint = 0; // Default
  regulator.hysteresis = 4;
  regulator.k = 50; // Нужно подгонять в случае неправильной работы регулятора (было 0.5)
  //****************************************
  //  Настраиваем Таймер2 для организации прерываний
  //****************************************
  Timer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer2.setPeriod(1000.0F);       // in microseconds
  Timer2.setCompare(TIMER_CH1, 1); // overflow might be small
  Timer2.attachInterrupt(TIMER_CH1, ISRenc);

  //****************************************
  // Блок жидкого меню
  //****************************************
  initLCDmenu();
}

void initLCDmenu()
{ // Настройка всех пунктов меню и инициализация в setup()
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("ПИВНАЯ СТАНЦИЯ");
  lcd.setCursor(0, 1);
  lcd.print("ДЛЯ САФФРОНА | v0.1");
  lcd.setCursor(1, 3);
  lcd.print("Kim Czulik  2020r.");
  delay(5000);
  mainMenu.init();
  //-------Добавление строк меню
  screenMainMenu.add_line(MainMenuLine1);
  screenMainMenu.add_line(MainMenuLine2);
  screenMainMenu.add_line(MainMenuLine3);
  screenMainMenu.add_line(MainMenuLine4);
  screenMainMenu.add_line(MainMenuLine5);

  screenSettingsZatirka.add_line(ZatirkaSettingsLine1);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine2);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine3);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine4);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine5);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine6);
  screenSettingsZatirka.add_line(ZatirkaSettingsLine7);

  screenSettingsWarka.add_line(WarkaSettingsLine1);
  screenSettingsWarka.add_line(WarkaSettingsLine2);
  screenSettingsWarka.add_line(WarkaSettingsLine3);
  screenSettingsWarka.add_line(WarkaSettingsLine4);
  screenSettingsWarka.add_line(WarkaSettingsLine5);
  screenSettingsWarka.add_line(WarkaSettingsLine6);
  screenSettingsWarka.add_line(WarkaSettingsLine7);
  screenSettingsWarka.add_line(WarkaSettingsLine8);

  screenRunWarka.add_line(runWarkaLine1);
  screenRunWarka.add_line(runWarkaLine2);
  screenRunWarka.add_line(runWarkaLine3);
  screenRunWarka.add_line(runWarkaLine4);
  screenRunWarka.add_line(runWarkaLine5);
  screenRunWarka.add_line(runWarkaLine6);
  screenRunWarka.add_line(runWarkaLine7);
  screenRunWarka.add_line(runWarkaLine8);

  screenRunZatirka.add_line(runZatirkaLine1);
  screenRunZatirka.add_line(runZatirkaLine2);
  screenRunZatirka.add_line(runZatirkaLine3);
  screenRunZatirka.add_line(runZatirkaLine4);
  screenRunZatirka.add_line(runZatirkaLine5);
  screenRunZatirka.add_line(runZatirkaLine6);
  screenRunZatirka.add_line(runZatirkaLine7);
  screenRunZatirka.add_line(runZatirkaLine8);

  // ---- Добавление функций (привязка) на строки. Передаём как void указатель на функцию принимающую void.
  MainMenuLine1.attach_function(3, funcSettingsZatirkaMenuOnCheck); //Функция номер 3 - выполнение
  MainMenuLine2.attach_function(3, startZatirkiMenuOnCheck);        // Если выбираем долгим нажатием строку - запуск функции
  MainMenuLine3.attach_function(3, funcSettingsWarkaMenuOnCheck);
  MainMenuLine4.attach_function(3, startWarkiMenuOnCheck);
  MainMenuLine5.attach_function(1, blankFunction); // Оставить так для возможности перемещения по меню

  //ZatirkaSettingsLine1.attach_function(1, blankFunction);                  //  "Настройки затирки");
  ZatirkaSettingsLine2.attach_function(1, zatirkaStupienSelectorIncrease); //"Ступень #" +1,
  ZatirkaSettingsLine2.attach_function(2, zatirkaStupienSelectorDecrease); //"Ступень #" -1,
  ZatirkaSettingsLine3.attach_function(1, zatirkaSetTimeIncrease);         //"Уст.время:",+
  ZatirkaSettingsLine3.attach_function(2, zatirkaSetTimeDecrease);         //"Уст.время:",-
  ZatirkaSettingsLine4.attach_function(1, zatirkaTemperSetpointIncrease);  //"Уст.темпер:",+
  ZatirkaSettingsLine4.attach_function(2, zatirkaTemperSetpointDecrease);  //"Уст.темпер:",-
  ZatirkaSettingsLine5.attach_function(1, zatirkaPumpWorkTimeIncrease);    //"Уст.врем.раб.помпы минут:",+
  ZatirkaSettingsLine5.attach_function(2, zatirkaPumpWorkTimeDecrease);    //"Уст.врем.раб.помпы минут:",-
  ZatirkaSettingsLine6.attach_function(1, zatirkaPumpStopTimeIncrease);    //"Уст.врем.отдыха помпы минут:"
  ZatirkaSettingsLine6.attach_function(2, zatirkaPumpStopTimeDecrease);    //"Уст.врем.отдыха помпы минут:"
  ZatirkaSettingsLine7.attach_function(3, backToMainMenu);                 //"ВЗАД");

  //WarkaSettingsLine1.attach_function(1, blankFunction);               //"Настройки варки"
  WarkaSettingsLine2.attach_function(1, warkaSetTimeIncrease);        //"Уст.общее время варки:",+
  WarkaSettingsLine2.attach_function(2, warkaSetTimeDecrease);        //"Уст.общее время варки:",-
  WarkaSettingsLine3.attach_function(1, warkaTemperSetpointIncrease); //"Уст.темпер:",+
  WarkaSettingsLine3.attach_function(2, warkaTemperSetpointDecrease); //"Уст.темпер:",-
  WarkaSettingsLine4.attach_function(1, warkaModeSelectorIncrease);   //"Заброс #",+
  WarkaSettingsLine4.attach_function(2, warkaModeSelectorDecrease);   //"Заброс #",-
  WarkaSettingsLine5.attach_function(1, warkaStupienSetTimeIncrease); //"Минут на ступень варки:",+
  WarkaSettingsLine5.attach_function(2, warkaStupienSetTimeDecrease); //"Минут на ступень варки:",-
  WarkaSettingsLine6.attach_function(1, warkaPumpWorkTimeIncrease);   //"Уст.врем.раб.помпы:",+
  WarkaSettingsLine6.attach_function(2, warkaPumpWorkTimeDecrease);   //"Уст.врем.раб.помпы:",-
  WarkaSettingsLine7.attach_function(1, warkaPumpStopTimeIncrease);   //"Уст.врем.отдыха помпы:",+
  WarkaSettingsLine7.attach_function(2, warkaPumpStopTimeDecrease);   //"Уст.врем.отдыха помпы:",-
  WarkaSettingsLine8.attach_function(3, backToMainMenu);              //"ВЗАД"

  //runWarkaLine1.attach_function(1, blankFunction);  //ПРОЦЕСС ВАРКИ
  runWarkaLine2.attach_function(1, blankFunction);  //"Ном. заброса: "
  runWarkaLine3.attach_function(1, blankFunction);  //"Темпер = "
  runWarkaLine4.attach_function(1, blankFunction);  //"До конца ступени:"
  runWarkaLine5.attach_function(1, blankFunction);  //"Мин. от начала работы:"
  runWarkaLine6.attach_function(1, blankFunction);  //Помпа работает?
  runWarkaLine7.attach_function(3, stopKran);       //"СТОП-КРАН"
  runWarkaLine8.attach_function(3, backToMainMenu); //"ВЗАД"

  //runZatirkaLine1.attach_function(1, blankFunction);  //ПРОЦЕСС ЗАТИРКИ
  runZatirkaLine2.attach_function(1, blankFunction);  //"Ном.ступени:"
  runZatirkaLine3.attach_function(1, blankFunction);  //"Темпер = "
  runZatirkaLine4.attach_function(1, blankFunction);  // "t.Setpoint:"
  runZatirkaLine5.attach_function(1, blankFunction);  //"До конца: "
  runZatirkaLine6.attach_function(1, blankFunction);  //Помпа работает?
  runZatirkaLine7.attach_function(3, stopKran);       // "СТОП-КРАН"
  runZatirkaLine8.attach_function(3, backToMainMenu); //"ВЗАД"

  screenMainMenu.set_displayLineCount(4); // Число линий на дисплее (rows)
  screenSettingsZatirka.set_displayLineCount(4);
  screenSettingsWarka.set_displayLineCount(4);
  screenRunWarka.set_displayLineCount(4);
  screenRunZatirka.set_displayLineCount(4);

  mainMenu.add_screen(screenMainMenu);        // 1
  mainMenu.add_screen(screenSettingsWarka);   // 2
  mainMenu.add_screen(screenSettingsZatirka); // 3
  mainMenu.add_screen(screenRunWarka);        // 4
  mainMenu.add_screen(screenRunZatirka);      // 5

  // // Set the number of decimal places for a "line".
  MainMenuLine5.set_decimalPlaces(3);
  runZatirkaLine4.set_decimalPlaces(1);
  runWarkaLine4.set_decimalPlaces(1);
  runWarkaLine5.set_decimalPlaces(1);

  mainMenu.set_focusPosition(Position::RIGHT);
  mainMenu.set_focusedLine(0);
  mainMenu.update();
}

void runStation()
{
  // User выбирает режим.
  // Режим ЗАТИРАНИЕ (1)
  // Установить время(длительность) работы и температуру, которую необходимо поддерживать. Сделать массив этих режимов.
  // Звуковой сигнал по окончании режима и переход к следующему.
  // Необходим регулятор - взял гуверовский релейный регулятор. Нужно настраивать коэф-ты

  // Режим ВАРКА (2)
  // Установка температуры и времени варки, время отсчёта и звуковой сигнал для закидывания порций хмеля.
  // Варка 60минут при 90град, хмель1 - на 15 минуте, хмель2 - на 30 минуте,хмель3 - на 55 минуте, конец варки - сигнал.

  // Режим работы помпы для обоих режимов станции, время работы и время отдыха.
  // Также добавить функцию сброса всех переменных предыдущего процесса при смене юзером режима работы (затирка/варка)

  filteredTemperature = filterKalman.filtered(thermistor->readCelsius()); // Сначала прочитаем температуру и отфильтруем значение фильтром Калмана

  userIO();
  switch (stationCurrentMode) //         СОСТОЯНИЕ СТАНЦИИ
  {
  case OFF:
    // Ничего не делать.
    break;
  case ZATIRANIE:
    zatiranie();
    break;
  case WARKA:
    warka();
    break;

  default:
    break;
  }
}

void userIO() // Логика взаимодействия с юзером, интерфейс
{
  // Отображение на дисплее выбора режима работы, настройки переменных режима.
  if (encoder.isRelease())
  {
    mainMenu.switch_focus();
    mainMenu.update();
  }
  if (encoder.isRight()) // Вызываем функцию под номером 1 на выделенной строке (увеличить)
  {
    mainMenu.call_function(1);
    mainMenu.update();
  }
  if (encoder.isLeft())
  {
    mainMenu.call_function(2); // Вызываем функцию под номером 2 на выделенной строке (уменьшить)
    mainMenu.update();
  }

  if (encoder.isHolded()) // Удержание - Выполнение задачи
  {
    mainMenu.call_function(3);
    mainMenu.update();
  }

  static uint32_t menuUpdateVarMs = millis();
  if (millis() - menuUpdateVarMs > 1000)
  {
    mainMenu.softUpdate();
    menuUpdateVarMs = millis();
  }
}

void zatiranie()
{
  if (waitForNextZatMode())
  {
    if (zatModes[numZatMode].durationTimeMs == 0 || (numZatMode > MAX_ZATIR_MODES - 1)) // Если следующего режима затирки нет
    {
      isFirstRun = 1;
      numZatMode = 0;
      isZatirkaRunning = 0;
      regulator.output = 0;
      digitalWrite(SOLID_RELAY_HEATER_PIN, 0);
      runPumpZatirka(pump.stopFlag);
      // pumpModeFlag = 0;
      stationCurrentMode = OFF;
      isZatirkaRunning = 0;
      numZatMode = 0;
      regulator.output = 0;
      digitalWrite(SOLID_RELAY_HEATER_PIN, 0);
      runPumpZatirka(pump.stopFlag);
      for (int i = 0; i < 5; i++) // Сигнал об окончании варки
      {
        tone(ALARM_PIN, 660, 500);
        delay(300);
        tone(ALARM_PIN, 770, 500);
        delay(300);
      }
    }
  }
}

void warka() // Логика работы помпы
{
  if (warkaDurationTimeMs > 0)
  {
    runPumpWarka(pump.runFlag); // Запускаем систему до тех пор пока время работы варки не вышло
    regulator.setpoint = warkaSetpointTemperature;
    regulator.input = filteredTemperature;
    
    digitalWrite(SOLID_RELAY_HEATER_PIN, regulator.getResultTimer());

    if (hmelZabrosTimeMsSinceStart[numVarZabros] > 0 && !(numVarZabros > MAX_WARKA_ZABROSY - 1))
    {                            //Существует ли текущий заброс?
      isTimeForNextHmel_Warka(); /* Проверка пришло ли время заброса? */
    }
  }
  else
  {
    isWarkaRunning = 0;
    regulator.output = 0;
    digitalWrite(SOLID_RELAY_HEATER_PIN, 0);
    stationCurrentMode = OFF;
    runPumpWarka(pump.stopFlag);
    //pumpModeFlag = 0;
    numVarZabros = 0;
    isFirstRun = 1;
    for (int i = 0; i < 5; i++) // Сигнал об окончании варки
    {
      tone(ALARM_PIN, 660, 500);
      delay(300);
      tone(ALARM_PIN, 770, 500);
      delay(300);
    }
  }
}

bool waitForNextZatMode() // Ожидание следующего режима затирки
{

  if (isFirstRun && !isZatirkaRunning) // Первый пробег
  {
    timeVarMs = millis(); // Сбросим таймер чтобы отсчёт до следующего таймаута начинался с нуля корректно.
    isZatirkaRunning = 1;
    isFirstRun = 0;
    regulator.setpoint = zatModes[numZatMode].temperature;
  }
  runPumpZatirka(pump.runFlag);
  regulator.input = filteredTemperature; 
  digitalWrite(SOLID_RELAY_HEATER_PIN, regulator.getResultTimer()); // отправляем на реле (ОС работает по своему таймеру)

  if (((millis() - timeVarMs) > (zatModes[numZatMode].durationTimeMs)) && isZatirkaRunning) // Время вышло - переходим на следующую ступень
  {
    isFirstRun = 1;
    isZatirkaRunning = 0;
    numZatMode++;

    // for (uint32_t i = 0; i < 1; i++)
    // {
    tone(ALARM_PIN, 660, 100);
    delay(150);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 510, 100);
    delay(100);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 770, 100);
    delay(550);
    tone(ALARM_PIN, 380, 100);
    //}
    return 1;
  }
  return 0;
}

void isTimeForNextHmel_Warka()
{
  if ((millis() - timeSinceStartOfMode) > (hmelZabrosTimeMsSinceStart[numVarZabros])) // Пришло время заброса?
  {
    numVarZabros++;

    tone(ALARM_PIN, 660, 100);
    delay(150);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 510, 100);
    delay(100);
    tone(ALARM_PIN, 660, 100);
    delay(300);
    tone(ALARM_PIN, 770, 100);
    delay(550);
    tone(ALARM_PIN, 380, 100);

    return;
  }
}

void runPumpZatirka(bool enableFlag) // Функция работы помпы при затирке
{
  pump.status = (!digitalRead(RELAY_PUMP_PIN));
  if (isZatirkaRunning && ((pump.timeInWorkMsZatirka == 0 && pump.timeInStopMsZatirka == 0) || (pump.timeInWorkMsZatirka == 0 && pump.timeInStopMsZatirka > 0)))
  {
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
    return;
  }

  if (enableFlag == false)
  {
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
    return;
  }

  static uint32_t pumpTimer = millis();

  if ((millis() - pumpTimer > pump.timeInWorkMsZatirka) && pump.dominantFlag)
  {
    pump.dominantFlag = 0;
    pumpTimer = millis();
  }

  if ((millis() - pumpTimer > pump.timeInWorkMsZatirka) && !pump.dominantFlag)
  {
    pump.dominantFlag = 1;
    pumpTimer = millis();
  }

  if (pump.dominantFlag)
    runPump(pump.runFlag, INVERSED_PUMP_LOGIC);
  if (!pump.dominantFlag)
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
}

void runPumpWarka(bool enableFlag = true) // Функция работы помпы при варке.
{
  pump.status = (!digitalRead(RELAY_PUMP_PIN));
  if (isWarkaRunning && ((pump.timeInWorkMsWarka == 0 && pump.timeInStopMsWarka == 0) || (pump.timeInWorkMsWarka == 0 && pump.timeInStopMsWarka > 0)))
  {
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
    return;
  }

  if (enableFlag == false)
  {
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
    return;
  }

  static uint32_t pumpTimer = millis();

  if ((millis() - pumpTimer > pump.timeInWorkMsWarka) && pump.dominantFlag)
  {
    pump.dominantFlag = 0;
    pumpTimer = millis();
  }

  if ((millis() - pumpTimer > pump.timeInWorkMsWarka) && !pump.dominantFlag)
  {
    pump.dominantFlag = 1;
    pumpTimer = millis();
  }

  if (pump.dominantFlag)
    runPump(pump.runFlag, INVERSED_PUMP_LOGIC);
  if (!pump.dominantFlag)
    runPump(pump.stopFlag, INVERSED_PUMP_LOGIC);
}

const char *isPumpRunning()
{
  if (pump.status)
    return "RUNNING";
  else
    return "  STOP ";
}

void ISRenc()
{
  encoder.tick();
}

const char *warkaStatus()
{
  if (stationCurrentMode == WARKA && isWarkaRunning)
    return "-WARKA IN PROCESS-";
  else
    return "   WARKA IS OVER    ";
}

const char *zatirkaStatus()
{
  if (stationCurrentMode == ZATIRANIE && isZatirkaRunning)
    return "-ZATIRKA IN PROCESS-";
  else
    return "  ZATIRKA IS OVER   ";
}

float returnTemperInCelsius()
{
  return filteredTemperature;
}

//****************************************
// Пользовательские функции интерфейса
//****************************************

//Главное меню
#pragma region mainFunctionsMenu
void funcSettingsZatirkaMenuOnCheck()
{
  mainMenu.change_screen(3);
  mainMenu.set_focusedLine(1);
  mainMenu.update();
}

void startZatirkiMenuOnCheck()
{
  if ((stationCurrentMode == OFF && !(isZatirkaRunning || isWarkaRunning)) && zatModes[0].durationTimeMs != 0)
  {
    tone(ALARM_PIN, 300, 300);
    delay(100);
    tone(ALARM_PIN, 300, 300);
    delay(100);
    tone(ALARM_PIN, 300, 300);
    stationCurrentMode = ZATIRANIE;
    numZatMode = 0;
    pump.dominantFlag = 1;
    timeSinceStartOfMode = millis();
    mainMenu.change_screen(5); //Переключиться на экран меню процесса затирки
    mainMenu.set_focusedLine(1);
    mainMenu.update();
  }
  if (isZatirkaRunning) // Если уже выполняется
  {
    mainMenu.change_screen(5); //Переключиться на экран меню процесса затирки
    mainMenu.set_focusedLine(1);
    mainMenu.update();
  }
}

void funcSettingsWarkaMenuOnCheck()
{
  mainMenu.change_screen(2);
  mainMenu.set_focusedLine(1);
  mainMenu.update();
}

void startWarkiMenuOnCheck()
{
  if (stationCurrentMode == OFF && !(isZatirkaRunning || isWarkaRunning) && hmelZabrosTimeMsSinceStart[0] != 0)
  {
    tone(ALARM_PIN, 300, 300);
    delay(200);
    tone(ALARM_PIN, 300, 300);
    delay(200);
    tone(ALARM_PIN, 300, 300);
    numVarZabros = 0;
    isWarkaRunning = 1;
    stationCurrentMode = WARKA;
    pump.dominantFlag = 1;
    timeSinceStartOfMode = millis();
    mainMenu.change_screen(4); //Переключиться на экран меню процесса варки
    mainMenu.set_focusedLine(1);
    mainMenu.update();
  }
  if (isWarkaRunning) // Если уже выполняется
  {
    mainMenu.change_screen(4); //Переключиться на экран меню процесса варки
    mainMenu.set_focusedLine(1);
    mainMenu.update();
  }
}
#pragma endregion

#pragma region ZatirkaFunctionsMenu // Меню настроек затирки (меню номер 3)
void zatirkaStupienSelectorIncrease()
{
  if (numZatMode >= 0 && numZatMode < 5)
  {
    numZatMode++;
  }
}
void zatirkaStupienSelectorDecrease()
{
  if (numZatMode > 0 && numZatMode < 6)
  {
    numZatMode--;
  }
}

uint32_t zatirkaTimeMins()
{
  return zatModes[numZatMode].durationTimeMs / (60 * 1000);
}

void zatirkaSetTimeIncrease()
{
  zatModes[numZatMode].durationTimeMs += 60 * 1000;
}

void zatirkaSetTimeDecrease()
{
  if (zatModes[numZatMode].durationTimeMs >= 60 * 1000)
    zatModes[numZatMode].durationTimeMs -= 60 * 1000;
}

void zatirkaTemperSetpointIncrease()
{
  zatModes[numZatMode].temperature++;
}

void zatirkaTemperSetpointDecrease()
{
  if (zatModes[numZatMode].temperature > 1)
  {
    zatModes[numZatMode].temperature--;
  }
}

uint32_t returnNumerStupieniZatirka()
{
  return numZatMode + 1;
}

int32_t returnZatirkaModeTemper()
{
  return zatModes[numZatMode].temperature;
}

uint32_t zatirkaPumpWorkTime()
{
  return pump.timeInWorkMsZatirka / (60 * 1000);
}

uint32_t zatirkaPumpStopTime()
{
  return pump.timeInStopMsZatirka / (60 * 1000);
}

void zatirkaPumpWorkTimeIncrease()
{
  pump.timeInWorkMsZatirka += 60 * 1000;
}

void zatirkaPumpWorkTimeDecrease()
{
  if (pump.timeInWorkMsZatirka >= 60 * 1000)
    pump.timeInWorkMsZatirka -= 60 * 1000;
}

void zatirkaPumpStopTimeIncrease()
{
  if (pump.timeInWorkMsZatirka > 0)
    pump.timeInStopMsZatirka += 60 * 1000;
}

void zatirkaPumpStopTimeDecrease()
{
  if (pump.timeInStopMsZatirka >= 60 * 1000)
    pump.timeInStopMsZatirka -= 60 * 1000;
}

void backToMainMenu()
{
  mainMenu.change_screen(1);
}
#pragma endregion

#pragma region WarkaFunctionsMenu // Меню настроек варки (меню номер 2)

uint32_t warkaTimeMins() // Вернуть общее время варки
{
  return warkaDurationTimeMs / (60 * 1000);
}

uint32_t returnWarkaTemperature()
{
  return warkaSetpointTemperature;
}

void warkaSetTimeIncrease()
{
  warkaDurationTimeMs += 60 * 1000;
}

void warkaSetTimeDecrease()
{
  if (warkaDurationTimeMs > 59999)
  {
    warkaDurationTimeMs -= 60 * 1000;
  }
}

void warkaTemperSetpointIncrease()
{
  warkaSetpointTemperature++;
}
void warkaTemperSetpointDecrease()
{
  if (warkaSetpointTemperature > 1)
    warkaSetpointTemperature--;
}

uint32_t returnNumerZabrosaWarka()
{
  return numVarZabros + 1;
}

void warkaSelectorIncrease()
{
  numVarZabros++;
}

void warkaModeSelectorIncrease()
{
  if (numVarZabros >= 0 && numVarZabros < MAX_WARKA_ZABROSY - 1)
  {
    numVarZabros++;
  }
}
void warkaModeSelectorDecrease()
{
  if (numVarZabros > 0 && numVarZabros < MAX_WARKA_ZABROSY)
  {
    numVarZabros--;
  }
}

uint32_t warkaStupienTime()
{
  return hmelZabrosTimeMsSinceStart[numVarZabros] / (60 * 1000);
}

void warkaStupienSetTimeIncrease()
{
  if ((numVarZabros > 0) && (hmelZabrosTimeMsSinceStart[numVarZabros] <= hmelZabrosTimeMsSinceStart[numVarZabros - 1]))
  {
    hmelZabrosTimeMsSinceStart[numVarZabros] = hmelZabrosTimeMsSinceStart[numVarZabros - 1] + (60 * 1000);
    return;
  }

  hmelZabrosTimeMsSinceStart[numVarZabros] += 60 * 1000;
}

void warkaStupienSetTimeDecrease()
{
  if ((numVarZabros > 0) && (hmelZabrosTimeMsSinceStart[numVarZabros] <= hmelZabrosTimeMsSinceStart[numVarZabros - 1]))
  {
    hmelZabrosTimeMsSinceStart[numVarZabros] = hmelZabrosTimeMsSinceStart[numVarZabros - 1] + (60 * 1000);
    return;
  }
  if (hmelZabrosTimeMsSinceStart[numVarZabros] > 59999)
  {
    hmelZabrosTimeMsSinceStart[numVarZabros] -= 60 * 1000;
  }
}

uint32_t warkaPumpWorkTime()
{
  return pump.timeInWorkMsWarka / (60 * 1000);
}

uint32_t warkaPumpStopTime()
{
  return pump.timeInStopMsWarka / (60 * 1000);
}

void warkaPumpWorkTimeIncrease()
{
  pump.timeInWorkMsWarka += 60 * 1000;
}

void warkaPumpWorkTimeDecrease()
{
  if (pump.timeInWorkMsWarka >= 60 * 1000)
    pump.timeInWorkMsWarka -= 60 * 1000;
}

void warkaPumpStopTimeIncrease()
{
  if (pump.timeInWorkMsWarka > 0)
    pump.timeInStopMsWarka += 60 * 1000;
}

void warkaPumpStopTimeDecrease()
{
  if (pump.timeInStopMsWarka >= 60 * 1000)
    pump.timeInStopMsWarka -= 60 * 1000;
}

void runPump(bool state, bool inverseFlag = INVERSED_PUMP_LOGIC) //Функция управления запуском помпы (инверсия или нормальный выход)
{
  if (inverseFlag == 1)
    digitalWrite(RELAY_PUMP_PIN, !state);
  if (inverseFlag == 0)
    digitalWrite(RELAY_PUMP_PIN, state);
}

#pragma endregion WarkaFunctionsMenu

#pragma region runWarka // Меню процесса варки (меню номер 4)

float timeUntilEndOfStageWarka()
{ //Время до конца текущей ступени варки
  float varWhichIsReturning = (float)((timeSinceStartOfMode + hmelZabrosTimeMsSinceStart[numVarZabros]) - millis()) / (60 * 1000);
  if (varWhichIsReturning < 600)
    return varWhichIsReturning;
  else
    return 0;
}

uint32_t timeSinceStartOfModeFunc()
{ //Время в минутах от начала варки вообще
  return (millis() - timeSinceStartOfMode) / (60 * 1000);
}

void stopKran()
{
  stationCurrentMode = OFF;
  isZatirkaRunning = 0;
  isWarkaRunning = 0;
  isFirstRun = 1;
  numVarZabros = 0;
  numZatMode = 0;
  isWarkaRunning = 0;
  regulator.output = 0;
  digitalWrite(SOLID_RELAY_HEATER_PIN, 0);
  runPumpWarka(pump.stopFlag);
  runPumpZatirka(pump.stopFlag);

  mainMenu.change_screen(1);
  tone(ALARM_PIN, 500, 3000);
}

#pragma endregion runWarka

#pragma region runZatirka // Меню процесса затирки (меню номер 5)

//Используются ранее написанные функции
float timeUntilEndOfZatirka()
{ //Время до конца текущей ступени затирки
  if (!isZatirkaRunning)
  {
    return 0;
  }

  float varWhichIsReturning = (float)((timeVarMs + zatModes[numZatMode].durationTimeMs) - millis()) / (60 * 1000);
  if (varWhichIsReturning < 600)
    return varWhichIsReturning;
  else
    return 0;
}

uint32_t returnNumerStupeniZatirki()
{
  return numZatMode + 1;
}

uint32_t returnSetpointTemperaStupeniZatirka()
{
  return zatModes[numZatMode].temperature;
}

#pragma endregion runZatirka


