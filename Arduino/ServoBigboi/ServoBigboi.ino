 #include <Servo.h> //используем библиотеку для работы с сервоприводом

Servo servo; //объявляем переменную servo типа Servo

void setup() //процедура setup

{
  
servo.attach(40); //привязываем привод к порту 10

}

void loop() //процедура loop

{

delay(2000); //ждем 2 секунды

servo.write(0); //ставим вал под 0





}
