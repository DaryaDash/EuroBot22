
#define ENCA 49

#define ENCB 48


int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

}


void loop() {
  readEncoder;
  Serial.print(pos);
  //Serial.println(pos);
//Serial.print("A");
 //Serial.print(digitalRead(ENCA)*5);
 //Serial.print(" B");
 //Serial.println(digitalRead(ENCB)*5);
 /*if (digitalRead(ENCA)>0){
   pos ++;
 }
 if (digitalRead(ENCB)>0){
   pos--;
 }
 Serial.println(pos);*/
 delay(10);
            }

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
                     }
 
