#include <ros.h>
#include <sensor_msgs/Range.h>//PING msgs
#include <std_msgs/Float32.h>//Motor msg
#include <std_msgs/Bool.h>        
#include <std_msgs/Int64.h> 
#include <Servo.h> 

Servo back_manipulator;
Servo back_servo;
Servo front_manipulator;
Servo front_servo;

float medianL(float newVal) {
  static int buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}
float medianF(float newVal) {
  static int buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}
float medianR(float newVal) {
  static int buf[3];
  static byte count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}

ros::NodeHandle nh;

  int irsensor= 7;  
  int sensorvalue; 

  int ENCA[]={19,2};//right left
  int ENCB[]={18,3};
  int pos[]={0,0};

  int MotorSpeed [] = {6,4,5}; //Right,Back,Left
  int MotorDirectionRight [] = {29,22,26};//Right,Back,Left
  int MotorDirectionLeft [] = {28,25,27};//Right,Back,Left

  int Start = 53;
  int failbut = 37;

  int PINGEcho[]={42,44,46};//left,front/right
  int PINGTrig[]={43,45,47};
  char *pingString[] = {" Left ","Front ", " Right "};


  sensor_msgs::Range rangeR_msg;
  sensor_msgs::Range rangeL_msg;
  sensor_msgs::Range rangeF_msg;

  std_msgs::Float32 ENCR_msg;
  std_msgs::Float32 ENCL_msg;
  std_msgs::Int64 line_msg;

  std_msgs::Bool start_msg;
  std_msgs::Bool back_but_msg;

  //ros::Publisher pub_line ("line", &line_msg);
  ros::Publisher pub_ENCR_POS ("ENCR_POS", &ENCR_msg);
  ros::Publisher pub_ENCL_POS ("ENCL_POS", &ENCL_msg);

  ros::Publisher pub_back_buttom ("fail", &back_but_msg);

  ros::Publisher pub_start ("start", &start_msg);

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


void messageCdRight (const std_msgs::Float32 &msg){
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
void messageCdLeft (const std_msgs::Float32 &msg){
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
void messageCdBack (const std_msgs::Float32 &msg){
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


void back_manipulcb (const std_msgs::Bool &msg){
  if (msg.data){
    back_manipulator.write(0);//открытый
  }
    else {
      back_manipulator.write(180);//закрытый
    }
  }
void back_servocb (const std_msgs::Bool &msg){
  if (msg.data){
    back_servo.write(10);// поднятие
  }
  else {
    back_servo.write(35);
  }
}
void front_manipulcb (const std_msgs::Bool &msg){
  if (msg.data){
    front_manipulator.write(0);// откр
  }
    else {
      front_manipulator.write(200);// закр
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
   front_servo.write(25);
   break;
   case 1:
   front_servo.write(60);
   break;
   case 2:
   front_servo.write(30);
   break;
 }}



ros::Subscriber<std_msgs::Bool> ENCZero("ENC_zero", &EncToZero);
ros::Subscriber<std_msgs::Bool> sub_front_manipul("front_grab", &front_manipulcb);
ros::Subscriber<std_msgs::Int64> sub_front_servo("front_servo", &front_servocb);
ros::Subscriber<std_msgs::Bool> sub_back_manipul("back_grab", &back_manipulcb);
ros::Subscriber<std_msgs::Bool> sub_back_servo("back_servo", &back_servocb);

ros::Subscriber<std_msgs::Float32> subR("v_right", &messageCdRight);
  ros::Subscriber<std_msgs::Float32> subL("v_left", &messageCdLeft);
  ros::Subscriber<std_msgs::Float32> subB("v_back", &messageCdBack);

 void EncToZero (const std_msgs::Bool &msg){
  if(msg.data==true){
  pos [0] =   0;
  pos [1] = 0;

  }
}

void setup() {
    back_manipulator.attach(38);
    back_servo.attach(39);
    front_manipulator.attach(40);
    front_servo.attach(41);
  nh.initNode();
  //attachInterrupt(digitalPinToInterrupt(ENCA[0]),readEncoder<0>,RISING);
  //attachInterrupt(digitalPinToInterrupt(ENCA[1]),readEncoder<1>,RISING);

  nh.subscribe(ENCZero);
  nh.subscribe(subL);
  nh.subscribe(subR);
  nh.subscribe(subB);
  nh.subscribe(sub_front_manipul);
  nh.subscribe(sub_front_servo);
  nh.subscribe(sub_back_manipul);
  nh.subscribe(sub_back_servo);
  nh.advertise(pub_ENCR_POS);
  nh.advertise(pub_ENCL_POS);
  nh.advertise(pub_start);
  nh.advertise(pub_range_front);
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);
  // nh.advertise(pub_line);
   nh.advertise(pub_back_buttom);
  // Serial.begin(9600);
  unsigned long echo;
  pinMode(failbut, INPUT);
  pinMode(Start, INPUT);
  pinMode(irsensor,INPUT);  
  //pinMode(IN_A0, INPUT);
  //pinMode(IN_D0, INPUT);
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
    //rangeL_msg.range= filter.filter(cm);
    rangeL_msg.range=medianL(cm);
    pub_range_left.publish (&rangeL_msg);
    break;
  case 1:
    //rangeF_msg.range= filter.filter(cm);
    rangeF_msg.range=medianF(cm);
    pub_range_front.publish (&rangeF_msg);
    break;
  case 2:
    //rangeR_msg.range= filter.filter(cm);
    rangeR_msg.range=medianR(cm);
    pub_range_right.publish (&rangeR_msg);
    break;
  }
  return cm ; //convert to CM then to inches

}

  int failstate;

void loop() {

  sensorvalue=digitalRead(irsensor); 
  line_msg.data=sensorvalue;
  //pub_line.publish (&line_msg);

  failstate = digitalRead(failbut);
  if (!failstate){
    back_but_msg.data = true;
    pub_back_buttom.publish (&back_but_msg);
  }
  else {
    back_but_msg.data = false;
    pub_back_buttom.publish (&back_but_msg);
  }

  int buttonState = digitalRead(Start);
  if (!buttonState){
    start_msg.data = true;
    pub_start.publish (&start_msg);
  }
  else {
    start_msg.data = false;
    pub_start.publish (&start_msg);
  }
  unsigned long ultrasoundValue;
  for(int i=0; i < 3; i++){
    ping(i); 

  }

//ENCL_msg.data=pos[0];
//pub_ENCL_POS.publish (&ENCL_msg);
//ENCR_msg.data=pos[1];
//pub_ENCR_POS.publish (&ENCR_msg);
  nh.spinOnce(); 
delay(10);
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
