
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
std_msgs::Float64 EnR_msg;
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
    
  }
  else{
     if (b=0) {
       posB=posB;
       EnB_msg.data = posB;
  
  }
  else {posB--;
   EnB_msg.data = posB;
  
  }
    if(l > 0){
    posL++;
     EnL_msg.data = posL;
   
  }
  else{
       if (l=0) {
       posL=posL;
        EnL_msg.data = posL;
      
  }
  else {posL--;
  
  }
  }
    if(r > 0){
    posR++;
    EnR_msg.data = posR;
    
  }
  else{
       if (r=0) {
         EnR_msg.data = posR;
  ;
  }
  else {posR--;
  EnR_msg.data = posR;
   }
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
 analogWrite(PIN_ENA, 80);
   analogWrite(PIN_ENC, 80); // Устанавливаем скорость 1-го мотора
   analogWrite(PIN_ENB, 80);
 // Устанавливаем скорость 1-го мотора
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
    pub_EnB.publish (&EnB_msg);
     pub_EnR.publish (&EnR_msg);
      pub_EnL.publish (&EnL_msg);
  // put your main code here, to run repeatedly:

}
