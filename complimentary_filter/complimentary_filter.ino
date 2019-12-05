#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include "pid.h"

LSM6 imu;
LIS3MDL mag;

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define BUZZER_PIN 6






void setup(){
  Serial.begin(9600);
  Wire.begin();
  // if statements to point out sensor problems
  if (!imu.init()){while (true){Serial.println("Failed to detect and initialize IMU!");}}
  if (!mag.init()){while (true){Serial.println("Failed to detect and initialize magnetometer!");}}
  // this is responsible for setting sensivity resolution.
  mag.enableDefault();
  imu.enableDefault();

  // CALIBRATE GYRO
  beep(); beep();
  delay(1000);
  calibrate_gyro();
  beep(); beep();
  delay(2000);

  // CALIBRATE MAGNETOMETER 
  beep(); beep(); beep();
  delay(1000);
  calibrate_magnetometer();
  beep(); beep(); beep();
}



// MAGNETOMETER - USEFUL ' X - AXIS '
// GUROSCOPE - USEFUL ' Z - AXIS ' 

void loop()
{
  mag.read();
  imu.read();

  update_readings();
  print_heading();
}


void update_readings(){
  
  // GYROSCOPE
  g_z = (imu.g.z - g_z_calibration) * 8.75;


  // MAGNETOMETER
  m_x = sensitivity_magnetometer * (mag.m.x - m_x_offset) * m_x_scale;
  m_x_output = ALPHA*x + (1-ALPHA)*m_x_output;



  // INTEGRATE GYRO READINGS
  float time_interval = millis() - previous_TimeStep_integral_update;
  previous_TimeStep_integral_update = millis();
  
  g_x_angle += update_integral(g_x, g_x_previous, time_interval);

  heading_complimentary_filter();

}


void heading_complimentary_filter(){
  heading = BETA * (heading + g_z_angle) + (1 - BETA) * m_x;
}

void print_heading(){
  Serial.print("Heading:  ");
  Serial.print(x_mag);
  Serial.print(" ");
  Serial.print(x_gyro);
  Serial.print(" ");
  Serial.println(heading);
}

}
