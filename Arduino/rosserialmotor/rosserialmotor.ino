#include <ros.h>
#include <geometry_msgs/Twist.h>

#define PIN_ENA 2 // Вывод управления скоростью вращения мотора №1
#define PIN_ENB 3 // Вывод управления скоростью вращения мотора №2
#define PIN_ENC 4 // Вывод управления скоростью вращения мотора №3
#define PIN_IN1 22 // Вывод управления направлением вращения мотора №1
#define PIN_IN2 23 // Вывод управления направлением вращения мотора №1
#define PIN_IN3 24 // Вывод управления направлением вращения мотора №2
#define PIN_IN4 25 // Вывод управления направлением вращения мотора №2
#define PIN_IN5 26 // Вывод управления направлением вращения мотора №3
#define PIN_IN6 27 // Вывод управления направлением вращения мотора №3

ros::NodeHandle nh;
int lowSpeed = 200;
int HightSpeed = 50;
double speed_ang=0, speed_lin=0;

geometry_msgs::Twist Twist_msg;

void messageCb( const geometry_msgs::Twist Twist_msg;){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &massageCb);
void Motors_init();
void MotorsL(int Pulse_Width1);
void  MotorsR(int Pulse_Width2);
void  MotorsB(int Pulse_Width3);

void stop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  digitalWrite(PIN_IN5, LOW);
  digitalWrite(PIN_IN6, LOW);
}

void setup() {
  nh.initNode();
  nh.advertise(twist_pub)
  nh.subscribe(sub);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_ENC, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_IN5, OUTPUT);
  pinMode(PIN_IN6, OUTPUT);
  stop();
}

void loop() {


}
