#include <ros.h>
#include <sensor_msgs/Range.h>//PING msgs
#include <std_msgs/Float64.h>//Motor msg
#include <std_msgs/Bool.h>        
#include <std_msgs/Int64.h> 
#include <Servo.h> 

Servo back_servo;
Servo front_manipulator;
Servo front_servo;

ros::NodeHandle nh;

  int ENCA[]={19,2};//right left
  int ENCB[]={18,3};
  int pos[]={0,0};

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


void messageCdRight (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(MotorSpeed [0], abs(msg.data));
  digitalWrite( MotorDirectionRight [0], HIGH);
  digitalWrite(MotorDirectionLeft [0], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [0],abs(msg.data));
  digitalWrite(MotorDirectionLeft [0], HIGH);
  digitalWrite(MotorDirectionRight [0], LOW);
  }
   if (msg.data==0) { 
  analogWrite(MotorSpeed [0],abs(msg.data));
  digitalWrite(MotorDirectionLeft [0], LOW);
  digitalWrite(MotorDirectionRight [0], LOW);
  }
};
void messageCdLeft (const std_msgs::Float64 &msg){
 if (msg.data<0) { 
  analogWrite(MotorSpeed [2], abs(msg.data));
  digitalWrite( MotorDirectionRight [2], HIGH);
  digitalWrite(MotorDirectionLeft [2], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [2], abs(msg.data));
  digitalWrite(MotorDirectionLeft [2], HIGH);
  digitalWrite(MotorDirectionRight [2], LOW);
  }
    if (msg.data==0) {   
  analogWrite(MotorSpeed [2],abs(msg.data));
  digitalWrite(MotorDirectionLeft [2], LOW);
  digitalWrite(MotorDirectionRight [2], LOW);
  }
};
void messageCdBack (const std_msgs::Float64 &msg){
  if (msg.data<0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite( MotorDirectionRight [1], HIGH);
  digitalWrite(MotorDirectionLeft [1], LOW);
  }
   if (msg.data>0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite(MotorDirectionLeft [1], HIGH);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
    if (msg.data==0) { 
  analogWrite(MotorSpeed [1], abs(msg.data));
  digitalWrite(MotorDirectionLeft [1], LOW);
  digitalWrite(MotorDirectionRight [1], LOW);
  }
};

void front_manipulcb (const std_msgs::Bool &msg){
  if (msg.data){
    front_manipulator.write(65);
  }
    else {
      front_manipulator.write(10);
    }
  }


void front_servocb (const std_msgs::Int64 &msg){
  /*if (msg.data == 0){
    front_servo.write(0);
  }
   else {
   if (msg.data == "take"){
     front_servo.write(65);
   }
    else {
        if (msg.data == "trow"){
     front_servo.write(40);
        }
     }
   }
  }*/


 switch (msg.data){
   case 0:
   front_servo.write(0);
   break;
   case 1:
   front_servo.write(65);
   break;
   case 2:
   front_servo.write(40);
   break;
 }}



ros::Subscriber<std_msgs::Bool> ENCZero("ENC_zero", &EncToZero);
  ros::Subscriber<std_msgs::Bool> sub_front_manipul("grab", &front_manipulcb);
ros::Subscriber<std_msgs::Int64> sub_front_servo("front_servo", &front_servocb);

ros::Subscriber<std_msgs::Float64> subR("v_right", &messageCdRight);
  ros::Subscriber<std_msgs::Float64> subL("v_left", &messageCdLeft);
  ros::Subscriber<std_msgs::Float64> subB("v_back", &messageCdBack);

 void EncToZero (const std_msgs::Bool &msg){
  if(msg.data==true){
  pos [0] =   0;
  pos [1] = 0;

  }
}

void setup() {
    front_servo.attach(41);
    front_manipulator.attach(40);
    back_servo.attach(39);
    
  nh.initNode();
  attachInterrupt(digitalPinToInterrupt(ENCA[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]),readEncoder<1>,RISING);

  nh.subscribe(ENCZero);
  nh.subscribe(subL);
  nh.subscribe(subR);
  nh.subscribe(subB);
  nh.subscribe(sub_front_manipul);
   nh.subscribe(sub_front_servo);
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
