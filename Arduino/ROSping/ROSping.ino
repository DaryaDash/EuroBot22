#include <ros.h>
#include <sensor_msgs/Range.h>
#define PIN_ECHOL 42
#define PIN_TRIGL 43

#define PIN_ECHOR 46
#define PIN_TRIGR 47

#define PIN_ECHOF 44
#define PIN_TRIGF 45

ros::NodeHandle nh;
sensor_msgs::Range rangeR_msg;
sensor_msgs::Range rangeL_msg;
sensor_msgs::Range rangeF_msg;
ros::Publisher pub_range_left ("range_left", &rangeL_msg);
ros::Publisher pub_range_right ("range_right", &rangeR_msg);
ros::Publisher pub_range_forward ("range_forward", &rangeF_msg);

long duration_right,duration_left,duration_forward, cm_right,cm_left,cm_forward;

void setup() {
  nh.initNode();
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);
  nh.advertise(pub_range_forward);
  Serial.begin (9600);
  //Определяем вводы и выводы
  pinMode(PIN_TRIGL, OUTPUT);
  pinMode(PIN_ECHOL, INPUT);
   pinMode(PIN_TRIGR, OUTPUT);
  pinMode(PIN_ECHOR, INPUT);
   pinMode(PIN_TRIGF, OUTPUT);
  pinMode(PIN_ECHOF, INPUT);
  // put your
}

void loop() {
  digitalWrite(PIN_TRIGF, LOW);
  digitalWrite(PIN_TRIGR, LOW);
  digitalWrite(PIN_TRIGL, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIGL, HIGH);
  digitalWrite(PIN_TRIGR, HIGH);
  digitalWrite(PIN_TRIGF, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGL, LOW);
  digitalWrite(PIN_TRIGF, LOW);
  digitalWrite(PIN_TRIGR, LOW);

  //  Время задержки акустического сигнала на эхолокаторе.
  duration_right = pulseIn(PIN_ECHOR, HIGH);
  duration_left =  pulseIn(PIN_ECHOL, HIGH);
  duration_forward = pulseIn(PIN_ECHOF, HIGH);

  // Теперь осталось преобразовать время в расстояние
  cm_right = (duration_right / 2) / 29.1; 
  cm_left=( duration_left/ 2)/ 29.1;
  cm_forward=(duration_forward/ 2)/ 29.1; // Сначала генерируем короткий импульс длительностью 2-5 микросекунд
 Serial.print("distance R: ");
  Serial.print(cm_right);
  Serial.println("cm.");
  Serial.print("distance L: ");
  Serial.print(cm_left);
  Serial.println("cm.");
  Serial.print("distance F: ");
  Serial.print(cm_forward);
  Serial.println("cm.");
  //*/
  rangeR_msg.range=cm_right;
  rangeL_msg.range=cm_left;
  rangeF_msg.range=cm_forward;
  pub_range_left.publish (&rangeL_msg);
  pub_range_right.publish (&rangeR_msg);
  pub_range_forward.publish (&rangeF_msg);
  nh.spinOnce();
  delay (10);  // put your main code here, to run repeatedly:

}
