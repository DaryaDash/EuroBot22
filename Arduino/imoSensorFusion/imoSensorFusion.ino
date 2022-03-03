// IMPORTANT
// this is not a working example, this is just to show how to set the library
// if you need a working example please see the other example

#include <Wire.h>
#include <FaBo9Axis_MPU9250.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include "SensorFusion.h" //SF
SF fusion;
BMP280 bmp280;
FaBo9Axis fabo_9axis;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;


void setup() {
  Serial.begin(115200);
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


  fabo_9axis.readAccelXYZ(&ax,&ay,&az);
  fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
  fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
 // fabo_9axis.readTemperature(&temp);
    bmp280.awaitMeasurement();



 static float meters, metersold;
  bmp280.getAltitude(meters);
 metersold = (meters);

  bmp280.triggerMeasurement();


  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  //choose only one of these two:
 // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  //pitch = fusion.getPitch();
  //roll = fusion.getRoll();    //you could also use getRollRadians() ecc    
  yaw = fusion.getYaw();

  //Serial.print("Pitch:\t"); Serial.println(pitch);
  //Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.print("gx: "); Serial.print(gx);   Serial.print("gy: "); Serial.print(gy); Serial.print("gz: "); Serial.println(gz);
   Serial.print("mx: "); Serial.print(mx);   Serial.print("my: "); Serial.print(my); Serial.print("mz: "); Serial.println(mz);
  Serial.println();
  delay (200);
}
