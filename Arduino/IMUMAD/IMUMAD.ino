#include <MadgwickAHRS.h>
#include <FaBo9Axis_MPU9250.h>
#include <Wire.h>

FaBo9Axis fabo_9axis;

#define TO_RAD 0.01745329252f

unsigned long tm, imu_t, prn_t;
const unsigned int imu_to = 20; // период обработки показаний датчиков
const unsigned int prn_to = 100; // период вывода информации в COM порт

float tdelta;
float ax, ay, az;
float gx_raw, gy_raw, gz_raw;

float imu[3];
float quat[4];
float e[3];

void setup() {
    Serial.begin(9600);
    Serial.println("Initializing I2C devices...");
    //accelgyro.initialize(); // инициализация датчиков
    if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while(1);
  
  }
   // Serial.println("Testing device connections...");
   // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
    // каждые 20мс вычисляем углы наклона
    tm = millis();
    if (imu_t + imu_to < tm) {
        tdelta = tm - imu_t; // вычисляем дельту времени в миллисекундах
        imu_t = tm;

        // запрашиваем данные у датчика MPU6050
       // accelgyro.getMotion6(&ax, &ay, &az, &gx_raw, &gy_raw, &gz_raw);
fabo_9axis.readAccelXYZ(&ax, &ay, &az);
  fabo_9axis.readGyroXYZ(&gx_raw, &gy_raw, &gz_raw);
  //fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
        // преобразуем сырые данные гироскопа в рад/с
        float gx = gx_raw * TO_RAD / 131.0;
        float gy = gy_raw * TO_RAD / 131.0;
        float gz = gz_raw * TO_RAD / 131.0;
 
        // вызываем алгоритм фильтра и передаем туда:
        // - дельту времени в секундах
        // - данные гироскопа в рад/с
        // - сырые данные акселерометра
        MadgwickAHRSupdateIMU(tdelta/1000.0, gx, gy, gz, (float)ax, (float)ay, (float)az);
        quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
        // преобразуем кватернион в углы Эйлера 
        quat2Euler(&quat[0], &imu[0]);
    }

    // каждые 100мс добавляем новую точку графика
    tm = millis();
    if (prn_t + prn_to < tm) {
        prn_t = tm;
        Serial.print(imu[0]/TO_RAD); Serial.print("\t");
        Serial.print(imu[1]/TO_RAD); Serial.print("\t");
        Serial.println(imu[2]/TO_RAD);

} }