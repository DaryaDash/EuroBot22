#include <ros.h>
#include <sensor_msgs/Range.h>

int PINGEcho[] = { 42, 44, 46 };  //left,front/right
int PINGTrig[] = { 43, 45, 47 };
char *pingString[] = { " Left ", "Front ", " Right " };

ros::NodeHandle nh;
sensor_msgs::Range rangeR_msg;
sensor_msgs::Range rangeL_msg;
sensor_msgs::Range rangeF_msg;
ros::Publisher pub_range_left("range_left", &rangeL_msg);
ros::Publisher pub_range_right("range_right", &rangeR_msg);
ros::Publisher pub_range_front("range_front", &rangeF_msg);

void setup() {
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);
  nh.advertise(pub_range_front);

  unsigned long echo;

  for (int i = 0; i < 3; i++) {
    pinMode(PINGTrig[i], OUTPUT);
    pinMode(PINGEcho[i], INPUT);
  }
}

unsigned long ping(int index) {
  unsigned long echo, cm;

  digitalWrite(PINGTrig[index], LOW);
  delayMicroseconds(5);
  digitalWrite(PINGTrig[index], HIGH);


  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PINGTrig[index], LOW);
  echo = pulseIn(PINGEcho[index], HIGH);
  cm = (echo / 2) / 29.1;
  switch (index) {
    case 0:
      rangeL_msg.range = cm;
      pub_range_left.publish(&rangeL_msg);
      break;
    case 1:
      rangeF_msg.range = cm;
      pub_range_front.publish(&rangeF_msg);
      break;
    case 2:
      rangeR_msg.range = cm;
      pub_range_right.publish(&rangeR_msg);
      break;
  }
  return cm;  //convert to CM then to inches
}

void loop() {
  unsigned long ultrasoundValue;
  for (int i = 0; i < 3; i++) {
    ping(i);
    delay(50);
  }
  nh.spinOnce();
  delay(50);
  // put your main code here, to run repeatedly:
}
