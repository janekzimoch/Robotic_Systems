#ifndef _Magentometer
#define _Magentometer

#include <LIS3MDL.h>
#define BUZZER_PIN      6   // To make the annoying beeping
const int NUM_CALIBRATIONS_MAG = 300;

class Magnetometer
{
    public:
        void init();
        void  readRaw();
        void  readCalibrated();
        void  calibrate();
        void  calculateOffsets();
        void  set_sensitivity();
        LIS3MDL mag;
        float cx = 0;
        float cy = 0;
        float cz = 0;
        float rx = 0;
        float ry = 0;
        float rz = 0;
        float rad_scale = 0;

    private:
        float sensitivity = 1;
        int x_min = 32767;
        int y_min = 32767;
        int z_min = 32767;
        int x_max = -32768;
        int y_max = -32768;
        int z_max = -32768;
        float x_offset = 0;
        float y_offset = 0;
        float z_offset = 0;
        float x_scale = 0;
        float y_scale = 0;
        float z_scale = 0;

};

void Magnetometer::init()
{
    if (!mag.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    mag.enableDefault();
}

void Magnetometer::readRaw()
{
  
  mag.read();
  
  rx = mag.m.x;
  ry = mag.m.y;
  rz = mag.m.z;
  
}

void Magnetometer::readCalibrated()
{

  mag.read();
  
  cx = sensitivity * (mag.m.x - x_offset) * x_scale;
  cy = sensitivity * (mag.m.y - y_offset) * y_scale;
  cz = sensitivity * (mag.m.z - z_offset) * z_scale;
  
}

void Magnetometer::calibrate()
{
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
  for (int i=0;i<NUM_CALIBRATIONS_MAG;i++)
  {
    
    mag.read();

    x_max = max(x_max, mag.m.x);
    y_max = max(y_max, mag.m.y);
    z_max = max(z_max, mag.m.z);

    x_min = min(x_min, mag.m.x);
    y_min = min(y_min, mag.m.y);
    z_min = min(z_min, mag.m.z);

    delay(50);
  }

  calculateOffsets();

  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  
}

void Magnetometer::calculateOffsets()
{
  
  x_offset = (x_max + x_min) / 2;
  y_offset = (y_max + y_min) / 2;
  z_offset = (z_max + z_min) / 2;

  x_scale = (x_max - x_min) / 2;
  y_scale = (y_max - y_min) / 2;
  z_scale = (z_max - z_min) / 2; 

  float avg_scale = (x_scale + y_scale + z_scale) / 3;

  x_scale = avg_scale / x_scale;
  y_scale = avg_scale / y_scale;
  z_scale = avg_scale / z_scale;


  
  Serial.print("MinMax Z: ");
  Serial.print(z_min);  
  Serial.print(", ");
  Serial.println(z_max);
  
  Serial.print("Offsets X: ");
  Serial.print(x_offset);  
  Serial.print(" Y: ");
  Serial.print(y_offset);
  Serial.print(" Z: ");
  Serial.println(z_offset);
  
  Serial.print("Scales X: ");
  Serial.print(x_scale);  
  Serial.print(" Y: ");
  Serial.print(y_scale);
  Serial.print(" Z: ");
  Serial.println(z_scale);
  
}


#endif
