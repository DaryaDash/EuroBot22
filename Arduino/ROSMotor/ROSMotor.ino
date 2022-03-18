#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define PIN_ENR 2 // Вывод управления скоростью вращения мотора №1
#define PIN_ENB 3 // Вывод управления скоростью вращения мотора №2
#define PIN_ENL 4 // Вывод управления скоростью вращения мотора №3
#define PIN_INRR 26 // Вывод управления направлением вращения мотора №1
#define PIN_INRL 27 // Вывод управления направлением вращения мотора №1
#define PIN_INLL 28 // Вывод управления направлением вращения мотора №2
#define PIN_INLR 29 // Вывод управления направлением вращения мотора №2
#define PIN_INBR 22 // Вывод управления направлением вращения мотора №3
#define PIN_INBL 25 // Вывод управления направлением вращения мотора №3

 ros::NodeHandle nh;  
void messageCdRight (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(PIN_ENR, abs(msg.data));
  digitalWrite(PIN_INRR, HIGH);
  digitalWrite(PIN_INRL, LOW);
  }
   if (msg.data>0) { 
  analogWrite(PIN_ENR, msg.data);
  digitalWrite(PIN_INRL, HIGH);
  digitalWrite(PIN_INRR, LOW);
  }
    if (msg.data==0) { 
  analogWrite(PIN_ENR, msg.data);
  digitalWrite(PIN_INRL, LOW);
  digitalWrite(PIN_INRR, LOW);
  }
};
void messageCdLeft (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(PIN_ENL, abs(msg.data));
  digitalWrite(PIN_INLR, HIGH);
  digitalWrite(PIN_INLL, LOW);
  }
   if (msg.data>0) { 
  analogWrite(PIN_ENL, msg.data);
  digitalWrite(PIN_INLL, HIGH);
  digitalWrite(PIN_INLR, LOW);
  }
    if (msg.data==0) { 
  analogWrite(PIN_ENL, msg.data);
  digitalWrite(PIN_INLL, LOW);
  digitalWrite(PIN_INLR, LOW);
  }

};
void messageCdBack (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(PIN_ENB, abs(msg.data));
  digitalWrite(PIN_INBR, LOW);
  digitalWrite(PIN_INBL, HIGH);
  }
   if (msg.data>0) { 
  analogWrite(PIN_ENB, msg.data);
  digitalWrite(PIN_INBL, HIGH);
  digitalWrite(PIN_INBR, LOW);
  }
    if (msg.data==0) { 
  analogWrite(PIN_ENB, msg.data);
  digitalWrite(PIN_INBL, LOW);
  digitalWrite(PIN_INBR, LOW);
  }
 
};

 ros::Subscriber<std_msgs::Float64> subR("v_right", &messageCdRight);
 ros::Subscriber<std_msgs::Float64> subL("v_left", &messageCdLeft);
 ros::Subscriber<std_msgs::Float64> subB("v_back", &messageCdBack);


void setup() {
  // put your setup code here, to run once:
pinMode(PIN_ENR, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_ENL, OUTPUT);
  pinMode(PIN_INRR, OUTPUT);
  pinMode(PIN_INRL, OUTPUT);
  pinMode(PIN_INLL, OUTPUT);
  pinMode(PIN_INLR, OUTPUT);
  pinMode(PIN_INBR, OUTPUT);
  pinMode(PIN_INBL, OUTPUT);
  // Команда остановки двум моторам
  digitalWrite(PIN_INRR, LOW);
  digitalWrite(PIN_INRL, LOW);
  digitalWrite(PIN_INBL, LOW);
  digitalWrite(PIN_INBR, LOW);
  digitalWrite(PIN_INLR, LOW);
  digitalWrite(PIN_INLL, LOW);

  nh.initNode();
  nh.subscribe(subL);
  nh.subscribe(subR);
 nh.subscribe(subB);

}

void loop() {
  nh.spinOnce();
  delay(1);
  // put your main code here, to run repeatedly:

}
