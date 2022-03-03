#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"

BMP280 bmp280;
FaBo9Axis fabo_9axis;

void setup() {
  Serial.begin(9600);
  Serial.println("RESET");
  Serial.println();

    Serial.print("Probe BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor found");
    else
    {
        Serial.println("Sensor missing");
        while (1) {}
    }

    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
  Serial.println("configuring device.");

  if (fabo_9axis.begin()) {
    Serial.println("configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("device error");
    while(1);
  
  }
}

void loop() {
  float ax,ay,az;
  float gx,gy,gz;
  float mx,my,mz;
  //float temp;

  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
 // fabo_9axis.readTemperature(&temp);
    bmp280.awaitMeasurement();

  //float temperature;
  //bmp280.getTemperature(temperature);

  //float pascal;
 // bmp280.getPressure(pascal);

 // static float meters, metersold;
 // bmp280.getAltitude(meters);
 // metersold = (meters);

  bmp280.triggerMeasurement();
    
  //Serial.print("AccX: ");
 // Serial.print(ax);
 // Serial.print(" AccY: ");
 // Serial.print(ay);
 // Serial.print(" AccZ: ");
 // Serial.println(az);
  
/*
  Serial.print("GyX: ");
  Serial.print(gx);
  Serial.print(" GyY: ");
  Serial.print(gy);
  Serial.print(" GyZ: ");
  Serial.println(gz);
 */
  Serial.print("MagX: ");
   Serial.print(mx);
 Serial.print(" MagY: ");
  Serial.print(my);
  Serial.print(" MagZ: ");
 Serial.println(mz);

 // Serial.print("Temp: ");
 // Serial.println(temp);

  //Serial.print(" HeightPT1: ");
  ///Serial.print(metersold);
 // Serial.print(" m; Height: ");
 // Serial.print(meters);
 // Serial.print(" Pressure: ");
 // Serial.print(pascal);
 // Serial.print(" Pa; T: ");
 // Serial.print(temperature);
 // Serial.println(" C");

  delay(1000);
}