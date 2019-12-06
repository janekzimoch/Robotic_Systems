#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"
#include "EEPROM.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15



// EEPROM SET UP
#include <EEPROM.h>
int addr = 0;
int val  ;
int value = 0;

uint8_t EEPROM_address = 0;
uint8_t EEPROM_addressX = 1;
uint8_t EEPROM_addressY = 2;

float lasttimereading = 0;


// CONSTANT
const int WHEEL_SPEED = 20;

// LINE SENSOR
float Kp_lineSensor = 0.01; //Proportional gain for position controller
float Kd_lineSensor = 0.0; //Derivative gain for position controller
float Ki_lineSensor = 0.0; //Integral gain for position controller
PID lineSensor_PID(Kp_lineSensor, Ki_lineSensor, Kd_lineSensor); //Position controller for left wheel position


unsigned long elapsed_time;
unsigned long last_time_check;
volatile long previous_count_right;
volatile long previous_count_left;

bool before_the_line = true;




// GLOBAL VARIABLES - REQUIRED FOR LINE FOLLOWING
float previous_reading_sum;
float readings_sum;


int romi_phase;  // variable for deciding what action to do next


// You may need to change these depending on how you wire
// in your line sensor.
#define LINE_LEFT_PIN A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A4 //Pin for the right line sensor

LineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

Kinematics kinematics;  // New kinematics class instance.

#define BAUD_RATE = 115200;

void setupMotorPins(){
  // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW );
}

void calibrateLineSensor(){
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();
  analogWrite(6, 100);
  delay(50);
  analogWrite(6, 0);
}


void setup() 
{

  setupEncoder0();
  setupEncoder1();
  
  calibrateLineSensor();

  romi_phase = 0;
  
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
}



void loop() 
{

  kinematics.update_ICC(count_right, count_left);
  
  switch (romi_phase)
  {
      case 0: 
        Serial.println("follow_line");
        follow_line();
        break;

      case 1: 
        Serial.println("finished");
        break;

      // we may want to make robi write to a .txt file such that it is easier to process data and store it. 
      case 2:
        Serial.println("prinitng EPROM values");
        printfromEEPROM();
        break;
  }
  delay(50);
}



void follow_line(){

  // READ THE LINE SENSOR
  float left_lineSensor = line_left.read_calibrated();
  if (left_lineSensor < 0) {left_lineSensor=0;}
  float centre_lineSensor = line_centre.read_calibrated();
  if (centre_lineSensor < 0){centre_lineSensor=0;}
  float right_lineSensor = line_right.read_calibrated();
  if (right_lineSensor < 0) {right_lineSensor=0;}

  previous_reading_sum = readings_sum;
  readings_sum = (left_lineSensor+right_lineSensor+centre_lineSensor);
  int score_lineSensor;
  float pid_output_lineSensor;


  // On the line
  if (readings_sum < 450){
    before_the_line = false;

    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(L_DIR_PIN, LOW);
    
    float left_probability = (float) left_lineSensor / readings_sum;
    float centre_probability = (float) centre_lineSensor / readings_sum;
    float right_probability = (float) right_lineSensor / readings_sum;

    score_lineSensor = (1000*left_probability) + (2000*centre_probability) + (3000*right_probability) - 2000;

    pid_output_lineSensor = lineSensor_PID.update(0, score_lineSensor);
    
    if (score_lineSensor < 0) {  // we are on the right of the line
      analogWrite( L_PWM_PIN , WHEEL_SPEED - (-pid_output_lineSensor));
      analogWrite( R_PWM_PIN, WHEEL_SPEED + (-pid_output_lineSensor));
    }
    else{      
      analogWrite( R_PWM_PIN, WHEEL_SPEED - pid_output_lineSensor);
      analogWrite( L_PWM_PIN , WHEEL_SPEED + pid_output_lineSensor);
    } 
  }
  
  else{
    if (before_the_line){
      analogWrite( R_PWM_PIN, 20);
      analogWrite( L_PWM_PIN , 20);
    }
    else{ // you finsished
      analogWrite( R_PWM_PIN, 0);
      analogWrite( L_PWM_PIN , 0);
      datatoEEPROM();

      romi_phase = 1;
    }
  }
}



void datatoEEPROM() {
  EEPROM.write(EEPROM_addressX, kinematics.getX());
  EEPROM.write(EEPROM_addressY, kinematics.getY());
  
  EEPROM_addressX +=2;
  EEPROM_addressY +=2;
}

void printfromEEPROM (){
  int z=1;

  EEPROM_addressX =1;
  EEPROM_addressY =2;
  while ( z < 50){
    Serial.println("X=");
    Serial.println(EEPROM.read(EEPROM_addressX));  
    Serial.println(",");
    Serial.println("Y=");
    Serial.println(EEPROM.read(EEPROM_addressY));  
    Serial.println(",");
    Serial.println("MEASUREMENT COUNT=");
    Serial.println(z);
    
    z++;
    EEPROM_addressX +=2;
    EEPROM_addressY +=2;
  }
}
