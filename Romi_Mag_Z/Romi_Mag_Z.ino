/*
 * After beeping once, wave the robot around, rotate it,
 * this will calibrate min/max values on the magnetometer.
 * After the next beeps, leave the robot still on a flat
 * surface, this will calibrate the gyro/accelerometer.
 * 
 * It should then be ready to go.
 * 
 */


#include "encoders.h"   // setup and isr to manage encoders.
#include "kinematics.h" // calculates x,y,theta from encoders.

#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"

#define BUZZER_PIN      6   // To make the annoying beeping
Kinematics  RomiPose;                                       // Not using ICC.
Imu imu;
Magnetometer mag; // Class for the magnetometer

float gyro_z_angle = 0;
float mag_z_angle = 0;

// Complementary filters.
float comp1 = 0;
float comp1_alpha = 0.2;



unsigned long update_t;

void setup() {
  
  pinMode( BUZZER_PIN, OUTPUT );


  Wire.begin();   

  mag.init();
  mag.calibrate();

  imu.init();
  imu.calibrate();

  // Start up the serial port.
  Serial.begin(9600);
  delay(1000);

  
  // Begin tracking encoder changes.
  setupEncoder0();
  setupEncoder1();

  update_t = millis();
  delay(1); // stop dt being = 0

  // First sensor update
  updateSensors();

  // The magnetometer has a fixed value based
  // on the earths magnetic field, so lets set
  // the gyro and kinematics to begin matching
  // the magnetometer instead.
  gyro_z_angle = mag_z_angle;
  RomiPose.setPose(0,0, gyro_z_angle);
  
  Serial.println("Starting:");
  Serial.print( mag_z_angle );
  Serial.print(",");
  Serial.print( gyro_z_angle );
  Serial.print(",");
  Serial.println( RomiPose.theta );
}

// Function wraps up the gyro, acceleromter (IMU) and
// magnetometer reads.  
void updateSensors() {
  
  long diff = ( millis() - update_t);
  update_t = millis();
  float dt = (float)diff;
  dt /= 1000;

  imu.readCalibrated();
  mag.readCalibrated();

  // Scale factor to rads - I found this by testing
  // it is still a bit off.  I've been adjusting the 
  // whole number to get better accuracy.
  float rad_scale = 900 *15.7;
  
  gyro_z_angle -= ((imu.gz / (rad_scale) ) * dt );

  
  // Extracted from: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  // Note that, assuming the Romi is flat, therefore pitch and roll both = 0
  // So I cancelled out most the terms
  // Left in the variable names so we can find it in the doc.
  // I'm not actually sure why this works so well - but there are still some
  // problems to solve.
  float Bfy = mag.cy;
  float Bfx = mag.cx;
  mag_z_angle = atan2( Bfy, Bfx );

}

void loop() {
  // put your main code here, to run repeatedly:
  RomiPose.update(e0_count, e1_count);
  
  updateSensors();

  // lets attempt combining the gyro and mag with
  // a simple complementary filter.
  // Basically, which do we trust more?
  // No prediction here, so no adjustment of alpha.
  comp1 = ( mag_z_angle * comp1_alpha) + ( gyro_z_angle * (1-comp1_alpha)  );

  
  
  Serial.print( comp1 );
  Serial.print(",");
  Serial.print( mag_z_angle );
  Serial.print(",");
  Serial.print( gyro_z_angle );
  Serial.print(",");
  Serial.println( RomiPose.theta );
  
  delay(25);

  

}

