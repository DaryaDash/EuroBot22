
#define INT_A_PIN 2

#define INT_B_PIN 3

#define SIG_A_PIN 4

#define SIG_B_PIN 5

long encA = 0;

long encB = 0;

void setup() {

pinMode(INT_A_PIN, INPUT);

pinMode(INT_B_PIN, INPUT);

pinMode(SIG_A_PIN, INPUT);

pinMode(SIG_B_PIN, INPUT);

Serial.begin(9600);

attachInterrupt(digitalPinToInterrupt(INT_A_PIN), EncA, RISING);

attachInterrupt(digitalPinToInterrupt(INT_B_PIN), EncB, RISING);

}

void EncA()

{

if ( digitalRead(SIG_A_PIN) )

{

encA++;

} else {

encA--;

}

}

void EncB()

{

if ( digitalRead(SIG_A_PIN) )

{

encB--;

} else {

encB++;

}

}

void loop() {

Serial.print(encA);

Serial.print("\t");

Serial.println(encB);

}
