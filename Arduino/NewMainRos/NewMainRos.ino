#include <ros.h>
#include <sensor_msgs/Range.h>//PING msgs
#include <std_msgs/Float64.h>//Motor msg
#include <std_msgs/Bool.h>         // navX-Sensor Register Definition header file




ros::NodeHandle nh;

int ENCA[]={19,2};//right left
int ENCB[]={18,3};
float mm = 0;
float pos[]= {0,0};
float CurrentAngle=0;

int MotorSpeed [] = {6,4,5}; //Right,Back,Left
int MotorDirectionRight [] = {29,22,26};//Right,Back,Left
int MotorDirectionLeft [] = {28,25,27};//Right,Back,Left

int PINGEcho[]={42,44,46};//left,front/right
int PINGTrig[]={43,45,47};
char *pingString[] = {" Left ","Front ", " Right "};


sensor_msgs::Range rangeR_msg;
sensor_msgs::Range rangeL_msg;
sensor_msgs::Range rangeF_msg;

std_msgs::Float64 ENCR_msg;
std_msgs::Float64 ENCL_msg;

ros::Publisher pub_ENCR_POS ("ENCR_POS", &ENCR_msg);
ros::Publisher pub_ENCL_POS ("ENCL_POS", &ENCL_msg);

ros::Publisher pub_range_left ("range_left_ping", &rangeL_msg);
ros::Publisher pub_range_right ("range_right_ping", &rangeR_msg);
ros::Publisher pub_range_front ("range_front_ping", &rangeF_msg);

  // switch (j){
  //   case 0:
  //   ENCR_msg.data=pos[j];
  //   pub_ENCR_POS.publish (&ENCR_msg);
  //   //Serial.println("Cas0");
  //   break;
  //    case 1:
  //   ENCL_msg.data=pos[j];
  //   pub_ENCR_POS.publish (&ENCL_msg);
  //  //Serial.println("Cas1");
  //   break;
  // }
void EncToZero (float pos []){
  pos [0] = 0;
  pos [1] = 0;
}

float EncTomm(float pos [], float mm, int enc){
  mm = abs(pos [enc]*0.02) ;
  return mm;
}

int right_vel= 155;
int left_vel= 155;
int forward_vel= 0;
ros::Subscriber<std_msgs::Float64> subX("linear_x", &messageLinearX);
ros::Subscriber<std_msgs::Float64> subY("linear_y", &messageLinearY);
ros::Subscriber<std_msgs::Float64> subtargyaw("target_yaw", &messageTargetAngularZ);
ros::Subscriber<std_msgs::Float64> subyaw("yaw", &messageAngularZ);

ros::Subscriber<std_msgs::Bool> substop("stop", &StopCb);

void StopCb (const std_msgs::Bool &msg){
EncToZero(pos);
  if (msg.data == true){
  analogWrite(MotorSpeed [0], 0);
  analogWrite(MotorSpeed [2], 0);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
}
void messageLinearY (const std_msgs::Float64 &msg){
EncToZero(pos);
  if (msg.data>0) {

  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], HIGH);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
  if (msg.data<0) {
  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], HIGH);
  }
}


void messageLinearX (const std_msgs::Float64 &msg){
EncToZero(pos);
  if (msg.data>0) {
  if (EncTomm(pos, mm, 1)< abs(msg.data)){
  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], HIGH);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], HIGH);
  }
    else {
 analogWrite(MotorSpeed [0], 0);
  analogWrite(MotorSpeed [2], 0);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
  }
  if (msg.data<0) {
    if (EncTomm(pos, mm, 1)< abs(msg.data)){
  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], HIGH);
  digitalWrite(MotorDirectionRight [2], HIGH);
  digitalWrite(MotorDirectionLeft [2], LOW);}
  else {
 analogWrite(MotorSpeed [0], 0);
  analogWrite(MotorSpeed [2], 0);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], LOW);    
  }
  }
  }

void messageAngularZ (const std_msgs::Float64 &msg){
  CurrentAngle = msg.data;
}
void messageTargetAngularZ (const std_msgs::Float64 &msg){
 
  while (CurrentAngle<abs(msg.data)){
  if (msg.data>0) {
  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [1], HIGH);
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  digitalWrite(MotorDirectionLeft [2], HIGH);
  }
  if (msg.data<0) {
  analogWrite(MotorSpeed [0], right_vel);
  analogWrite(MotorSpeed [2], left_vel);
  analogWrite(MotorSpeed [1], 0);
  digitalWrite(MotorDirectionRight [0], LOW);
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  digitalWrite(MotorDirectionLeft [1], HIGH);
  digitalWrite(MotorDirectionRight [2], HIGH);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
  }
  
  }

void setup() {
nh.initNode();
attachInterrupt(digitalPinToInterrupt(ENCA[0]),readEncoder<0>,RISING);
attachInterrupt(digitalPinToInterrupt(ENCA[1]),readEncoder<1>,RISING);

nh.subscribe(subX);
nh.subscribe(subY);
nh.subscribe(subyaw);
nh.subscribe(subtargyaw);
nh.subscribe(substop);
nh.advertise(pub_ENCR_POS);
nh.advertise(pub_ENCL_POS);
nh.advertise(pub_range_front);
nh.advertise(pub_range_left);
nh.advertise(pub_range_right);
// Serial.begin(9600);
unsigned long echo;

for(int i=0; i < 3; i++) {
pinMode(MotorDirectionRight[i], OUTPUT);
pinMode(MotorDirectionLeft[i], OUTPUT);
pinMode(MotorSpeed[i], OUTPUT);
digitalWrite(MotorDirectionLeft[i], LOW);
digitalWrite(MotorDirectionRight[i], LOW);
pinMode(PINGTrig[i], OUTPUT);
pinMode(PINGEcho[i], INPUT);
}
for(int i=0; i < 2; i++) {
pinMode(ENCA[i], INPUT);
pinMode(ENCB[i], INPUT);
  }

}


unsigned long ping(int index){
unsigned long echo,cm;

  digitalWrite(PINGTrig[index], LOW);
  delayMicroseconds(5);
  digitalWrite(PINGTrig[index], HIGH);


  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PINGTrig[index], LOW);
  echo = pulseIn(PINGEcho[index], HIGH);
  cm = (echo / 2) / 29.1; 
    switch (index){
    case 0:
    rangeL_msg.range=cm;
    pub_range_left.publish (&rangeL_msg);
    break;
  case 1:
    rangeF_msg.range=cm;
    pub_range_front.publish (&rangeF_msg);
    break;
  case 2:
    rangeR_msg.range=cm;
    pub_range_right.publish (&rangeR_msg);
    break;
  }
  return cm ; //convert to CM then to inches

}
//Ping loop
void loop() {
  unsigned long ultrasoundValue;
  for(int i=0; i < 3; i++){
    ping(i); 

  }
ENCL_msg.data=pos[0];
pub_ENCL_POS.publish (&ENCL_msg);
ENCR_msg.data=pos[1];
pub_ENCR_POS.publish (&ENCR_msg);
  nh.spinOnce(); 

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
