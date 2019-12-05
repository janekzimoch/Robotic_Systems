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

double ALPHA = 0.2;
double BETA = 0.9;

float previous_TimeStep_integral_update;

// PID for gyro-heading
float Kp_heading = 0.00001; //Proportional gain 
float Kd_heading = 0; //Derivative gain 
float Ki_heading = 0; //Integral gain 
PID heading_gyro_PID(Kp_heading, Ki_heading, Kd_heading ); // controller for left wheel 

// sensor fussion
long heading;

// accelerometer - linear acceleration (mg)
long a_x;
long a_y;
long a_z;

// gyro - angular acceleration (mdps)
long g_x;
long g_y;
long g_z;
long g_x_calibration;
long g_y_calibration;
long g_z_calibration;
long g_x_previous;
long g_y_previous;
long g_z_previous;

// gyro - heading angle (milli degrees)
long g_x_angle;
long g_y_angle;
long g_z_angle;
long g_x_angle_output;  // low-pass filter applied
long g_y_angle_output;
long g_z_angle_output;

// magnetometer - gauss
long m_x;
long m_y;
long m_z;
long scale_x;  // for magnetometer calibration
long scale_y;
long scale_z;
long offset_x;  // for magnetometer calibration
long offset_y;
long offset_z;

long m_x_output;
long m_y_output;
long m_z_output;


void setup(){
  Serial.begin(9600);
  Wire.begin();

  analogWrite(6, 10);
  delay(50);
  analogWrite(6, 0);
  delay(200);

  if (!imu.init())
  {
    while (true){
        Serial.println("Failed to detect and initialize IMU!");
    }
  }
  
  if (!mag.init())
  {
    while (true){
      Serial.println("Failed to detect and initialize magnetometer!");
    }
  }

  mag.enableDefault();
  imu.enableDefault(); // this is responsible for setting sensivity. You can increase range of readings as a tradeoff of resolution.

  calibrate_gyro();
  delay(2000);
//  calibrate_magnetometer();
}





void loop()
{

  update_readings();
  
//  resist_rotation_gyro(g_z_angle);

//  resist_rotation_mag();

  print_accelerometer();
//  print_gyro();
//  print_magnetometer();
//  print_mixed();

  digitalWrite( R_DIR_PIN, LOW );
  digitalWrite( L_DIR_PIN, LOW );

  analogWrite( R_PWM_PIN, 20);
  analogWrite( L_PWM_PIN, 20);

  delay(100);
}


float update_integral(float current, float previous, float time_interval_milis){
  return (current + previous)/2 * (time_interval_milis*0.001);
}

void update_readings(){
  imu.read();
  mag.read();
  
  // ACCELEROMETER - convert to 'mg' units (milli gravity)
  a_x = imu.a.x * 0.061;
  a_y = imu.a.y * 0.061;
  a_z = imu.a.z * 0.061;

  // GYRO - convert to 'mdps' units (milli degrees per second)
  g_x = (imu.g.x - g_x_calibration) * 8.75;
  g_y = (imu.g.y - g_y_calibration) * 8.75;
  g_z = (imu.g.z - g_z_calibration) * 8.75;

  // MAGNETOMETER - convert to 'gauss' units 
  m_x = scale_x * (mag.m.x - offset_x) * 6.842; //0.00014616;
  m_x_output = ALPHA*m_x + (1-ALPHA)*m_x_output;
  
  m_y = scale_y * (mag.m.y - offset_y) * 6.842; //0.00014616;
  m_y_output = ALPHA*m_y + (1-ALPHA)*m_y_output;
  
  m_z = scale_z * (mag.m.z - offset_z) * 6.842; //0.00014616;
  m_z_output = ALPHA*m_z + (1-ALPHA)*m_z_output;



  float time_interval = millis() - previous_TimeStep_integral_update;
  previous_TimeStep_integral_update = millis();
  
  g_x_angle += update_integral(g_x, g_x_previous, time_interval);
  g_y_angle += update_integral(g_y, g_y_previous, time_interval);
  g_z_angle += update_integral(g_z, g_z_previous, time_interval);
  g_x_previous = g_x;
  g_y_previous = g_y;
  g_z_previous = g_z;

  g_x_angle_output = ALPHA*g_x_angle + (1-ALPHA)*g_x_angle_output; 
  g_y_angle_output = ALPHA*g_y_angle + (1-ALPHA)*g_y_angle_output; 
  g_z_angle_output = ALPHA*g_z_angle + (1-ALPHA)*g_z_angle_output; 

  BETA = 0.4;
  heading = BETA * (heading + g_z_angle) + (1 - BETA) * m_y;


}




void resist_rotation_gyro(float g_z_angle){
    int heading_output;
    heading_output = heading_gyro_PID.update(0, g_z_angle);
    
    if (g_z_angle < 0){
      digitalWrite( R_DIR_PIN, LOW );
      digitalWrite( L_DIR_PIN, HIGH );
    }
    else{
      digitalWrite( R_DIR_PIN, HIGH );
      digitalWrite( L_DIR_PIN, LOW );
    }

    if (heading_output > 20 or heading_output < 20){
      heading_output = 20;
    }
    analogWrite( R_PWM_PIN, heading_output);
    analogWrite( L_PWM_PIN, heading_output);
}



void print_accelerometer(){
  
  // accelerometer prints accelerations
  Serial.print("A: ");
  Serial.print(a_x);  // forward-backward acceleration
  Serial.print(" ");
  Serial.print(a_y);  // left-right
  Serial.print(" ");
  Serial.println(a_z); // up-down
}
void print_gyro(){
  
  // gyro prints angular accelerations
  Serial.print("G:  ");
  Serial.print(g_x_angle_output);  // roll angle
  Serial.print(" ");
  Serial.print(g_y_angle_output); // pitch angle 
  Serial.print(" ");
  Serial.println(g_z_angle_output);  // yaw angle 
}
void print_magnetometer(){
  
  // magnetometer prints gauss (maxwell per square centimeter - how much magnetic field is in the given area (measures magentic flux)
  // magnetic field goes from north to south. so if you start turning area (in the magnetometer) 
  // through which the field passes will be changing [proportional to sin(pheta)]
  Serial.print("M:  ");
  Serial.print(m_x_output);
  Serial.print(" ");
  Serial.print(m_y_output);
  Serial.print(" ");
  Serial.println(m_z_output);
  
}
void print_mixed(){
  

//  Serial.print(m_x);
//  Serial.print(" ");
  Serial.print(m_y);
  Serial.print(" ");
  Serial.print(g_z_angle);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.println(m_y_output);
  
}




void calibrate_gyro(){
    
  // two beeps - gyro calibration started
  for( int count = 0; count < 2; count++ ) {
    analogWrite(6, 10);
    delay(50);
    analogWrite(6, 0);
    delay(200);
  }
    
  int calibration_length = 200;
  float total_g_x = 0;
  float total_g_y = 0;
  float total_g_z = 0;

  for( int count = 0; count < calibration_length; count++ ) {
    imu.read();
    
    total_g_x += imu.g.x;
    total_g_y += imu.g.y;
    total_g_z += imu.g.z;
    delay(20);
  }
  g_x_calibration = total_g_x / calibration_length;
  g_y_calibration = total_g_y / calibration_length;
  g_z_calibration = total_g_z / calibration_length;

  // two beeps - gyro calibration completed
  for( int count = 0; count < 2; count++ ) {
    analogWrite(6, 10);
    delay(50);
    analogWrite(6, 0);
    delay(200);
  }
}

void calibrate_magnetometer(){

  // 3 beeps - magnetometer calibration started
  for( int count = 0; count < 3; count++ ) {
    analogWrite(6, 10);
    delay(50);
    analogWrite(6, 0);
    delay(200);
  }

  // not sure why we set MIN / MAX values to these... it didn't work initially then i saw in the liblary that they implemented this iitial values and it helped.
  long min_x = 32767;
  long max_x = -32767;
  
  long min_y = 32767;
  long max_y = -32767;
 
  long min_z = 32767;
  long max_z = -32767;

  int calibration_length = 500;
  for( int count = 0; count < calibration_length; count++ ) {
   
    mag.read();

    max_x = max(max_y, mag.m.x);
    max_y = max(max_y, mag.m.y);
    max_z = max(max_z, mag.m.z);

    min_x = min(min_x, mag.m.x);
    min_y = min(min_y, mag.m.y);
    min_z = min(min_z, mag.m.z);

    
    Serial.print("M:  ");
    Serial.print(min_x);
    Serial.print(" ");
    Serial.print(max_x);
    Serial.print(" ");
    Serial.print(min_y);
    Serial.print(" ");
    Serial.print(max_y);
    Serial.print(" ");
    Serial.print(min_z);
    Serial.print(" ");
    Serial.println(max_z);
    
    delay(50);
  } 

  float range_x;
  float range_y;
  float range_z;
  
  offset_x = (max_x + min_x)/2;
  range_x = (max_x - min_x)/2;

  offset_y = (max_y + min_y)/2;
  range_y = (max_y - min_y)/2;

  offset_z = (max_z + min_z)/2;
  range_z = (max_z - min_z)/2;
  
  float avg_range;  
  avg_range = range_x + range_y + range_z;
  
  scale_x = avg_range / range_x;
  scale_y = avg_range / range_y;
  scale_z = avg_range / range_z;

  // three beeps - magnetometer calibration completed
  for( int count = 0; count < 3; count++ ) {
    analogWrite(6, 10);
    delay(50);
    analogWrite(6, 0);
    delay(200);
  }
}
