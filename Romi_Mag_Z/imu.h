#ifndef _IMU_h
#define _IMU_h

#include <LSM6.h>
#define BUZZER_PIN      6   // To make the annoying beeping
const int NUM_CALIBRATIONS_IMU = 50;

class Imu
{
    public:
        void init();
        void  readRaw();
        void  readCalibrated();
        void  calibrate();
        LSM6 imu;
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;

    private:
        float a_sensitivity = 0.061;
        float g_sensitivity = 8.75;
        float gx_offset = 0;
        float gy_offset = 0;
        float gz_offset = 0;


};

void Imu::init()
{
    if (!imu.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    imu.enableDefault();

  // I found the two below lines in another piece of
  // code using the LSM6.

  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);

  // Set the accelerometer full scale to 16 g because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL1_XL, 0b10000100);
}

void Imu::readRaw()
{
  
  imu.read();
  
  gx = imu.g.x;
  gy = imu.g.y;
  gz = imu.g.z;

  ax = imu.a.x;
  ay = imu.a.y;
  az = imu.a.z;
  
}

void Imu::readCalibrated()
{

  imu.read();
  
  gx = g_sensitivity * (imu.g.x - gx_offset);
  gy = g_sensitivity * (imu.g.y - gy_offset);
  gz = g_sensitivity * (imu.g.z - gz_offset);

  ax = a_sensitivity * imu.a.x;
  ay = a_sensitivity * imu.a.y;
  az = a_sensitivity * imu.a.z;

 
 
  
}

void Imu::calibrate()
{
   analogWrite(BUZZER_PIN, 10);
  delay(50);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  analogWrite(BUZZER_PIN, 10);
  delay(50);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  for (int i=0;i<NUM_CALIBRATIONS_IMU; i++)
  {
    //analogWrite(BUZZER_PIN, 10);
    delay(5);
    
    imu.read();

    gx_offset += ((float)imu.g.x / NUM_CALIBRATIONS_IMU);
    gy_offset += ((float)imu.g.y / NUM_CALIBRATIONS_IMU);
    gz_offset += ((float)imu.g.z / NUM_CALIBRATIONS_IMU);
    analogWrite(BUZZER_PIN, 0);
    delay(50);
  }

   analogWrite(BUZZER_PIN, 10);
  delay(50);
  digitalWrite( BUZZER_PIN, LOW );
  delay(50);
  analogWrite(BUZZER_PIN, 10);
  delay(50);
  digitalWrite( BUZZER_PIN, LOW );
  delay(50);

  
}




#endif

