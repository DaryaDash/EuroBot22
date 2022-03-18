
#define PIN_ENA 5 // Вывод управления скоростью вращения мотора №1
#define PIN_ENB 3 // Вывод управления скоростью вращения мотора №2
#define PIN_ENC 4 // Вывод управления скоростью вращения мотора №3
#define PIN_IN1 27 // Вывод управления направлением вращения мотора №1
#define PIN_IN2 29 // Вывод управления направлением вращения мотора №1
#define PIN_IN3 22 // Вывод управления направлением вращения мотора №2
#define PIN_IN4 24 // Вывод управления направлением вращения мотора №2
#define PIN_IN5 25 // Вывод управления направлением вращения мотора №3
#define PIN_IN6 28 // Вывод управления направлением вращения мотора №3

uint8_t power = 105; // Значение ШИМ (или скорости вращения)
void setup() {
  Serial.begin(9600);
  // Установка всех управляющих пинов в режим выхода
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_ENC, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
  // Команда остановки двум моторам
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_IN5, LOW);
  digitalWrite(PIN_IN6, LOW);

 }

void loop() {
  analogWrite(PIN_ENA, power);
   analogWrite(PIN_ENC, power); // Устанавливаем скорость 1-го мотора
   analogWrite(PIN_ENB, power); // Устанавливаем скорость 1-го мотора
    // Задаём направление для 1-го мотора
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    // Задаём направление для 2-го мотора
   digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
       // Задаём направление для 3-го мотора
    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    power += 30;
    Serial.println (power);
    delay(600); // Пауза 3 секунды
  
   }
  //power = 225;    
 /* // Вращаем моторы в другую сторону с разной скоростью
  while(power > 105) {
     analogWrite(PIN_ENA, power); // Устанавливаем скорость 1-го мотора
    analogWrite(PIN_ENB, power);
   analogWrite(PIN_ENC, power); // Устанавливаем скорость 2-го мотора
    // Задаём направление для 1-го мотора
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    // Задаём направление для 2-го мотора
    digitalWrite(PIN_IN3, LOW);
      digitalWrite(PIN_IN4, HIGH);
      // Задаём направление для 3-го мотора
    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    delay(3000); // Пауза 3 секунды
    power -= 30; // Уменьшаем скорость
  }
  power = 105;
}*//////////