
#define PIN_ENA 5 // Вывод управления скоростью вращения мотора л
#define PIN_ENB 6   // Вывод управления скоростью вращения мотора №2п
#define PIN_ENC 4 // Вывод управления скоростью вращения мотора №3з
#define PIN_IN1 26 // Вывод управления направлением вращения мотора №1л
#define PIN_IN2 27 // Вывод управления направлением вращения мотора №1л
#define PIN_IN3 29 // Вывод управления направлением вращения мотора №2п
#define PIN_IN4 28 // Вывод управления направлением вращения мотора №2п
#define PIN_IN5 22 // Вывод управления направлением вращения мотора №3
#define PIN_IN6 25 // Вывод управления направлением вращения мотора №3

int ENCA[]={19,2};//right left
int ENCB[]={18,3};
long pos[]={0,0};



//int pos = 0;




void setup() {
   Serial.begin(9600);
   pinMode(ENCA[0], INPUT);
   pinMode(ENCA[1], INPUT);
   pinMode(ENCB[0], INPUT);
   pinMode(ENCB[1], INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA[0]),readEncoder<0>,RISING);
attachInterrupt(digitalPinToInterrupt(ENCA[1]),readEncoder<1>,RISING);
 
  //pinMode(ENCA,INPUT);
  //pinMode(ENCB,INPUT);
  //attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
 pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_ENC, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
 analogWrite(PIN_ENA,255);
   analogWrite(PIN_ENC, 255); // Устанавливаем скорость 1-го мотора
   analogWrite(PIN_ENB, 255);
 // Устанавливаем скорость 1-го мотора
    // Задаём направление для 1-го мотора
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    // Задаём направление для 2-го мотора
   digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4,LOW);
       // Задаём направление для 3-го мотора
    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, LOW);

}

void loop() {


   digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    // Задаём направление для 2-го мотора
   digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4,HIGH);
       // Задаём направление для 3-го мотора
    digitalWrite(PIN_IN5, LOW);
    digitalWrite(PIN_IN6, HIGH);
    Serial.print(" R ");
    Serial.print(pos[0] );
    Serial.print(" L ");
    Serial.println(pos[1] );
  
}


 

template <int j>
void readEncoder(){

  int b = digitalRead(ENCB[j]);
  if(b > 0){
    pos[j]++;
    
  }
  else{
    pos[j]--;
  }

}