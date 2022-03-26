
#define ENCA 19

#define ENCB 18


int pos = 0;

void readEncoder(){
 
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
                     }
 

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);

}


void loop() {
Serial.println(pos);
 }
            


