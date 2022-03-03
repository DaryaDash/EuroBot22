void setup() {
  Serial.begin(9600);
  pinMode(3, INPUT_PULLUP);
}

bool flag = false;
uint32_t btnTimer = 0;
void loop() {
  // читаем инвертированное значение для удобства
  bool btnState = !digitalRead(3);
  if (btnState && !flag && millis() - btnTimer > 100) {
    flag = true;
    btnTimer = millis();
    Serial.println("press");
  }
  if (!btnState && flag && millis() - btnTimer > 100) {
    flag = false;
    btnTimer = millis();
    //Serial.println("release");
  }
}