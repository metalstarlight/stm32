#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306_STM32.h>

enum sex_t : uint8_t
{
  FEMALE,
  MALE
};

//Обьявляем структуру которая потом будет классом
class Person
{

public:
  Person(const char *name, uint8_t age, sex_t sex);
  void setPerson(char *name, uint8_t age, sex_t sex);
  void printPerson();
  void setName(const char *value);
  const char *getname()
  {
    return _name;
  };
  void setAge(uint16_t value);

private:
  char _name[30];
  uint8_t _age;
  sex_t _sex;
};

Person first("Jordan", 19, MALE);
Person sec("Ania", 20, FEMALE);
void setup()
{
  Serial.begin(115200);
  Serial.println();
}

///////////////////////////////////////////////////////////////////// ОСНОВНОЙ ЦИКЛ
/////////////////////////////////////////////////////////////////////

void loop()
{

  first.printPerson();
  sec.printPerson();

  delay(2000);
}

////////////////////////////////////////////////////////////////////// КОНЕЦ ОСНОВНОГО ЦИКЛА
/////////////////////////////////////////////////////////////////////

//Описываем конструктор
Person::Person(const char *name, uint8_t age, sex_t sex)
{
  setName(name);
  _age = age;
  _sex = sex;
}

//Описываем методы

void Person::setPerson(char *name, uint8_t age, sex_t sex)
{
  strncpy(_name, name, sizeof(_name) - 1);
  _name[sizeof(_name) - 1] = '\0';
  _age = age;
  _sex = sex;
}

void Person::printPerson()
{
  Serial.print("Name:   ");
  Serial.println(_name);

  Serial.print("Age:   ");
  Serial.println(_age);

  if (_sex == FEMALE)
  {
    Serial.print("Sex: FEMALE");
  }
  else
  {
    Serial.print("Sex: MALE");
  }
  Serial.println();
}

void Person::setAge(uint16_t value)
{
  _age = value;
}

void Person::setName(const char *value)
{
  if (value && *value)
  {
    strncpy(_name, value, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }
  else
  {
    _name[0] = '\0';
  }
}
