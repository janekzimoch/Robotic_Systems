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


const int NUM_CALIBRATIONS_GYRO = 200;
const int NUM_CALIBRATIONS_MAG = 300;

const double SENSITIVITY_GYRO = 8.75;
const double SENSITIVITY_MAG = 6.842; //0.00014616;

const double ALPHA = 0.3;
const double BETA = 0.9;


// MAGNETOMETER
long m_x;
long m_x_output;
long m_y;
long m_y_output;
long m_z;
long m_z_output;

int x_min = 32767;
int x_max = -32768;
int y_min = 32767;
int y_max = -32768;
int z_min = 32767;
int z_max = -32768;
float x_offset = 0;
float x_scale = 0;
float y_offset = 0;
float y_scale = 0;
float z_offset = 0;
float z_scale = 0;


// GYRO
long g_z;
long g_z_previous;
float g_z_calibration;
long g_z_angle;



// sensor fussion
long heading;


// turn function
bool rotation = true;
float previous_TimeStep_turn = 0;


// other
float previous_TimeStep_integral_update;


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


// We only use z-axis of magnetometer and z-axis of gyro for the heading. Other axes are not useful for 2D movement.
// MAGNETOMETER - USEFUL ' Z - AXIS '
// GYROSCOPE - USEFUL ' Z - AXIS ' 


// MAIN loop of the code. every 50ms reads values of the MAG and GYRO and prints heading info.
void loop()
{
  mag.read();
  imu.read();

  turn();
  update_readings();
  print_heading();
  delay(50);
}


// this function rotates robot. It is used to test how magnetometer readings change when motors work
void turn(){

  float time_elapsed = millis() - previous_TimeStep_turn;
  if (time_elapsed > 500){
    rotation = !rotation;
    time_elapsed = previous_TimeStep_integral_update;
    previous_TimeStep_turn = millis();
  }
  digitalWrite( R_DIR_PIN, rotation );
  digitalWrite( L_DIR_PIN, !rotation );

  analogWrite( R_PWM_PIN, 20);
  analogWrite( L_PWM_PIN, 20);
}


// uses readings of gyro and magnetometer to calculate change in heading
void update_readings(){
  
  // GYROSCOPE
  g_z = (imu.g.z - g_z_calibration) * SENSITIVITY_GYRO;


  // MAGNETOMETER
  m_x = SENSITIVITY_MAG * (mag.m.x - x_offset) * x_scale;
  m_x_output = ALPHA*m_x + (1-ALPHA)*m_x_output;
  m_y = SENSITIVITY_MAG * (mag.m.y - y_offset) * y_scale;
  m_y_output = ALPHA*m_y + (1-ALPHA)*m_y_output;
  m_z = SENSITIVITY_MAG * (mag.m.z - z_offset) * z_scale;
  m_z_output = ALPHA*m_z + (1-ALPHA)*m_z_output;



  // INTEGRATE GYRO READINGS
  float time_interval = millis() - previous_TimeStep_integral_update;
  previous_TimeStep_integral_update = millis();
  
  g_z_angle += update_integral(g_z, g_z_previous, time_interval);   // 90 degrees is around 50000 in magnitude 

  heading_complimentary_filter();

}


// Complimentary filter combines short term advantages of GYRO and long term advantages of MAGNETOMETER
void heading_complimentary_filter(){
  heading = BETA * (heading + g_z_angle) + (1 - BETA) * m_z;
}




void calibrate_gyro(){
    
  float total_g_z = 0;

  for( int count = 0; count < NUM_CALIBRATIONS_GYRO; count++ ) {
    imu.read();
    
    total_g_z += imu.g.z;
    delay(20);
  }
  g_z_calibration = total_g_z / NUM_CALIBRATIONS_GYRO;
}



// MAGNETOMETER calibration consists of two steps
// It is important to roatet magnetometer in multiple directions during calibration which should take around 10 seconds.
// this is imporant for MAG to accurately calculate x_scale and x_offset.
void calibrate_magnetometer(){

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
}



void calculateOffsets()
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
}


float update_integral(float current, float previous, float time_interval_milis){
  return (current + previous)/2 * (time_interval_milis*0.001);
}



void beep() {
  analogWrite(6, 80);
  delay(50);
  analogWrite(6, 0);
  delay(50);
}


void print_heading(){
  Serial.print("Heading:  ");
  Serial.print(m_x);
  Serial.print(" ");
  Serial.print(m_y);
  Serial.print(" ");
  Serial.println(m_z);
//  Serial.print(" ");
//  Serial.println(g_z_angle);
//  Serial.print(" ");
//  Serial.println(heading);
  
}
