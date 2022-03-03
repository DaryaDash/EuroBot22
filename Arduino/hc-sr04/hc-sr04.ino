#define PIN_TRIGL 43
#define PIN_ECHOL 42
#define PIN_TRIGR 45
#define PIN_ECHOR 44
#define PIN_TRIGF 45
#define PIN_ECHOF 44
long duration_R,duration_L, duration_F, cm_R, cm_L, cm_F;

void setup() {

  // Инициализируем взаимодействие по последовательному порту

  Serial.begin (9600);
  //Определяем вводы и выводы
  pinMode(PIN_TRIGR, OUTPUT);
  pinMode(PIN_ECHOR, INPUT);
   pinMode(PIN_TRIGL, OUTPUT);
  pinMode(PIN_ECHOL, INPUT);
   pinMode(PIN_TRIGF, OUTPUT);
  pinMode(PIN_ECHOF, INPUT);
}

void loop() {

  digitalWrite(PIN_TRIGR, LOW);
  digitalWrite(PIN_TRIGF, LOW);
  digitalWrite(PIN_TRIGL, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIGR, HIGH);
  digitalWrite(PIN_TRIGF, HIGH);
    digitalWrite(PIN_TRIGL, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGR, LOW);
  digitalWrite(PIN_TRIGF, LOW);
  digitalWrite(PIN_TRIGL, LOW);
  //  Время задержки акустического сигнала на эхолокаторе.
  duration_R = pulseIn(PIN_ECHOR, HIGH);
  duration_F = pulseIn(PIN_ECHOF, HIGH);
    duration_L = pulseIn(PIN_ECHOL, HIGH);
  // Теперь осталось преобразовать время в расстояние
  cm_R = (duration_R / 2) / 29.1;  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
 cm_F = (duration_F / 2) / 29.1;
 cm_L = (duration_L / 2) / 29.1;
  Serial.print("R: ");
  Serial.print(cm_R);
  Serial.println("cm."); 
  Serial.print("F: ");
  Serial.print(cm_F);
  Serial.println("cm.");
   Serial.print("L: ");
  Serial.print(cm_L);
  Serial.println("cm.");

  // Задержка между измерениями для корректной работы скеча
  delay(250);
}
