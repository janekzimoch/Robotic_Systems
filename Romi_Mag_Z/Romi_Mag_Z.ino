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
#define BUTTON_A       14   // Push button labelled A on board.

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FORWARD_ LOW
#define BACKWARD_ HIGH



Kinematics  RomiPose;                                       // Not using ICC.
Imu imu;
Magnetometer mag; // Class for the magnetometer

float gyro_z_angle = 0;
float mag_z_angle = 0;

float gyro_z_angle_delta = 0;

// Complementary filters.
float comp1 = 0;
float comp1_alpha = 0.1;
float comp2 = 0;
float comp2_beta = 0.99;  // increasing this value slows down the response of this filter. it's lagging. 

// GH filter
float gh_heading = 0;
float speed_kinematics = 0;
float gh_filter_tau = 0.3;
float gh_filter_kapa = 0.2;
float gh_prediction = 0.0;

float angular_velocity = 0;


unsigned long update_t;
unsigned long start_time = 0;

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
  comp1 = mag_z_angle;
  gh_heading = mag_z_angle;
  RomiPose.setPose(0,0, gyro_z_angle);
  
  Serial.println("Starting:");
  Serial.print( mag_z_angle );
  Serial.print(",");
  Serial.print( gyro_z_angle );
  Serial.print(",");
  Serial.println( RomiPose.theta );

  start_time = millis();
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
  float rad_scale_gyro = 900 * 16.024;

  gyro_z_angle_delta = ((imu.gz / (rad_scale_gyro) ) * dt );
  gyro_z_angle -= gyro_z_angle_delta;
  
  // Lets gyro_z_angle theta to keep it between 0 and TWO_PI
  // Not strictly necessary, but predictable at least.
  while( gyro_z_angle < -PI ) gyro_z_angle += TWO_PI;
  while( gyro_z_angle > PI ) gyro_z_angle -= TWO_PI;

  
  // Extracted from: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  // Note that, assuming the Romi is flat, therefore pitch and roll both = 0
  // So I cancelled out most the terms
  // Left in the variable names so we can find it in the doc.
  // I'm not actually sure why this works so well - but there are still some
  // problems to solve.
  float Bfz = mag.cz;
  float Bfx = mag.cx;
  float rad_scale_mag = 1.0;  // calibration of magnetometer doesn't help. because it is about the ratio of signals from z and x axis. and arctan already produces heading in radians.
  mag_z_angle = atan2(Bfx, Bfz) / rad_scale_mag;


  angular_velocity = (RomiPose.theta-RomiPose.last_theta) / diff;    
  speed_kinematics = (1 - gh_filter_tau) * speed_kinematics + (gh_filter_tau * angular_velocity);
  gh_prediction = gh_heading + ( speed_kinematics * diff );

}


void beep(){
  analogWrite(BUZZER_PIN, 10);
  delay(50);
  analogWrite(BUZZER_PIN, 0);
  delay(50);
}



void spin(){
  // spin clockwise for 20 seconds
  if(millis() - start_time < 19900){ // 19.2 seconds allow for three revolutions
    digitalWrite( L_DIR_PIN, FORWARD_);
    digitalWrite( R_DIR_PIN, BACKWARD_);

    analogWrite( L_PWM_PIN, 20 );
    analogWrite( R_PWM_PIN, 20 );    
  }
  else{
    analogWrite( L_PWM_PIN, 0 );
    analogWrite( R_PWM_PIN, 0 );
  } 
}



void loop() {
  // put your main code here, to run repeatedly:
  RomiPose.update(e0_count, e1_count);
  
  updateSensors();
  spin();

  // a simple complementary filter.
  comp1 = ( mag_z_angle * comp1_alpha) + ( gyro_z_angle * (1-comp1_alpha)  );

  // different type of complimentary filter - not yet verified whether it is better than 'comp1'
  comp2 = comp2_beta * (comp2 - gyro_z_angle_delta) + (1 - comp2_beta) * mag_z_angle;

  // gh filter
  gh_heading = gh_prediction + gh_filter_kapa * ( comp1 - gh_prediction ); 
  
  
  Serial.print( comp1 );  // complimentary filter
  Serial.print(",");
  Serial.print( gh_heading );  // gh filter
  Serial.print(",");
  Serial.print( mag_z_angle );  // magnetometer
  Serial.print(",");
  Serial.print( gyro_z_angle );  // gyroscope
  Serial.print(",");
  Serial.println( RomiPose.theta );  // wheel encoder odometry
  
  delay(25);
}
