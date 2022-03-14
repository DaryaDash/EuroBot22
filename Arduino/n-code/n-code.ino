
#include <ros.h>
#include <std_msgs/Float64.h>
#define PIN_ENA 2 // Вывод управления скоростью вращения мотора л
#define PIN_ENB 3   // Вывод управления скоростью вращения мотора №2п
#define PIN_ENC 4 // Вывод управления скоростью вращения мотора №3з
#define PIN_IN1 26 // Вывод управления направлением вращения мотора №1л
#define PIN_IN2 27 // Вывод управления направлением вращения мотора №1л
#define PIN_IN3 28 // Вывод управления направлением вращения мотора №2п
#define PIN_IN4 29 // Вывод управления направлением вращения мотора №2п
#define PIN_IN5 22 // Вывод управления направлением вращения мотора №3
#define PIN_IN6 25 // Вывод управления направлением вращения мотора №3

ros::NodeHandle nh;

int ENCA []={49,51,53}; /// left, back, right
int ENCB []={48,50,52}; /// left, back, right
int posL=0;
int posR=0;
int posB=0;
]std_msgs::Float64 EnR_msg;
std_msgs::Float64 EnL_msg;
std_msgs::Float64 EnB_msg;

ros::Publisher pub_EnR ("range_left_enc", &EnR_msg);
ros::Publisher pub_EnL ("range_right_enc", &EnL_msg);
ros::Publisher pub_EnB ("range_front_enc", &EnB_msg);

void readEncoder(){
  int b = digitalRead(ENCB[1]);
  int r = digitalRead(ENCB[2]);
  int l = digitalRead(ENCB[0]);
  if(b > 0){
    posB++;
    
  pub_range_front.publish (&EnB_msg);
  }
  else{
     if (b=0) {
       posB=posB;
       Serial.print(posB);
  Serial.println();
  }
  else {posB--;
  Serial.print(posB);
  Serial.println();
  }
    if(l > 0){
    posL++;
    Serial.print(posL);
   Serial.println();
  }
  else{
       if (l=0) {
       posL=posL;
       Serial.print(posL);
   Serial.println();
  }
  else {posL--;
  Serial.print(posL);
   Serial.println();
  }
  }
    if(r > 0){
    posR++;
    Serial.print(posR);
  Serial.println();
  }
  else{
       if (r=0) {
  Serial.print(posR);
  Serial.println();
  }
  else {posR--;
  Serial.print(posR);
  Serial.println();}
  }
}
}

void setup() {
nh.initNode();
nh.advertise(pub_EnR);
nh.advertise(pub_EnL);
nh.advertise(pub_EnB);
 for(int i=0; i < 3; i++) {
attachInterrupt(digitalPinToInterrupt( ENCA[i]), readEncoder, RISING);

attachInterrupt(digitalPinToInterrupt(ENCB[i]), readEncoder, RISING);
pinMode(ENCA[i], INPUT);
pinMode(ENCB[i], INPUT);
  }
    pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_ENC, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
 analogWrite(PIN_ENA, 160);
   analogWrite(PIN_ENC, 160); // Устанавливаем скорость 1-го мотора
   analogWrite(PIN_ENB, 160
  ); // Устанавливаем скорость 1-го мотора
    // Задаём направление для 1-го мотора
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    // Задаём направление для 2-го мотора
   digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4,HIGH);
       // Задаём направление для 3-го мотора
    digitalWrite(PIN_IN5, HIGH);
    digitalWrite(PIN_IN6, LOW);

  }


void loop() {
   readEncoder;
  // put your main code here, to run repeatedly:

}
