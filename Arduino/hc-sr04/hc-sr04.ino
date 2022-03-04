int PINGEcho[]={42,44,46};//left,front/right
int PINGTrig[]={43,45,47};
char *pingString[] = {" Left ","Front ", " Right "};
void setup() {
  // put your setup code here, to run once:
unsigned long echo;

 Serial.begin (9600);
 pinMode(PINGTrig[0], OUTPUT);
  pinMode(PINGEcho[0], INPUT);

  pinMode(PINGTrig[1], OUTPUT);
  pinMode(PINGEcho[1], INPUT);

  pinMode(PINGTrig[2], OUTPUT);
  pinMode(PINGEcho[2], INPUT);
  //Определяем вводы и выводы
}

unsigned long ping(int index)
{
  unsigned long echo,cm;

 digitalWrite(PINGTrig[index], LOW);
 

  delayMicroseconds(5);
  
  digitalWrite(PINGTrig[index], HIGH);


  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PINGTrig[index], LOW);
  echo = pulseIn(PINGEcho[index], HIGH);
  cm = (echo / 2) / 29.1; 
  return cm ; //convert to CM then to inches
}

void loop() {
   unsigned long ultrasoundValue;
  for(int i=0; i < 3; i++){
    ultrasoundValue = ping(i); 
    Serial.print(pingString[i]);
    Serial.print(ultrasoundValue);
    Serial.print("cm, ");    
    delay(50);

  }
  Serial.println();
  delay(50); 
  // put your main code here, to run repeatedly:

}
