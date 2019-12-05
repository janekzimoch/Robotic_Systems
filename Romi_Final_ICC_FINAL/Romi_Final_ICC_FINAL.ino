#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"

//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

float Kp_lineSensor = 0.01; //Proportional gain for position controller
float Kd_lineSensor = 0.0; //Derivative gain for position controller
float Ki_lineSensor = 0.0; //Integral gain for position controller
PID lineSensor_PID(Kp_lineSensor, Ki_lineSensor, Kd_lineSensor); //Position controller for left wheel position

float Kp_heading = 15; //Proportional gain 
float Kd_heading = 0; //Derivative gain 
float Ki_heading = 0; //Integral gain 
PID heading_PID(Kp_heading, Ki_heading, Kd_heading ); // controller for left wheel 

// POSITION
float Kp_position = 2.0; //Proportional gain for position controller
float Kd_position = 8.0; //Derivative gain for position controller
float Ki_position = 0.0; //Integral gain for position controller
PID position_PID(Kp_position, Ki_position, Kd_position); //Position controller for right wheel position


//// LEFT WHEEL - POSITION
//float Kp_left = 0.17; //Proportional gain for position controller
//float Kd_left = 2; //Derivative gain for position controller
//float Ki_left = 0.0; //Integral gain for position controller
//PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for right wheel position


// RIGHT WHEEL
float Kp_right = 34; //Proportional gain for position controller
float Kd_right = 20.0; //Derivative gain for position controller
float Ki_right = 0.5; //Integral gain for position controller
PID right_PID(Kp_right, Ki_right, Kd_right); //Position controller for right wheel position

// LEFT WHEEL
float Kp_left = 34; //Proportional gain for position controller
float Kd_left = 200.0; //Derivative gain for position controller
float Ki_left = 0.5; //Integral gain for position controller
PID left_PID(Kp_left, Ki_left, Kd_left); //Position controller for right wheel position

float speed_demand_RIGHT;
float speed_demand_LEFT;
float velocity_RIGHT;
float velocity_LEFT;

unsigned long elapsed_time;
unsigned long last_time_check;
volatile long previous_count_right;
volatile long previous_count_left;

float distance_to_home;
float distance_to_home_previous;
unsigned long time_stamp_for_distance;


float required_angle;
float heading_output;

// GLOBAL VARIABLES - REQUIRED FOR LINE TERMINATION
float previous_reading_sum;
int temp_X_origin;
int temp_Y_origin;
float temp_pheta;
float readings_sum;
bool just_lost_the_line = false;
bool second_turn_done = false;

byte r_pwm;
byte l_pwm;
bool onLine_past;
int last_High;  // =0 we are on the line, =1 we are one the left of the line, =2 we are on the right 

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

  r_pwm = 24;
  l_pwm = 24;
  romi_phase = 0;
  
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
//  kinematics.setX(1000);
//  kinematics.setY(-1000);
//  kinematics.setPheta(0.785);
}



void loop() 
{

  kinematics.update_ICC(count_right, count_left);

  Serial.print(kinematics.getX());
  Serial.print(", ");
  Serial.print(kinematics.getY());
  Serial.print(", ");
  Serial.println(kinematics.getPheta());
  


  
  
  switch (romi_phase)
  {
      case 0: 
        Serial.println("follow_line");
        follow_line();
        break;

      case 1: 
        Serial.println("terminate_line_follow");
        turn_to_temp_origin();
        break;
        
      case 2:
        analogWrite(6, 100);
        delay(2000);
        analogWrite(6, 0);
        romi_phase = 3;
        delay(2000);
        break;
        
      case 3:
        Serial.println("turn_home");
        turn_home();
        break;
        
      case 4:
        Serial.println("go_home");
        go_home_speed_control();
        //go_home();
        break;
        
      case 5:
        Serial.println("your_home");
        break;
  }

  delay(50);
}

bool sign(float num){
  if (num > 0) return true; // go backwards
  else return false; // go forwards
}

void go_home_speed_control(){

  
  
  int x_position = kinematics.getX();
  int y_position = kinematics.getY();
  if (millis() - time_stamp_for_distance > 300){
    if (distance_to_home == 0){
      distance_to_home_previous = 10000000;
    }
    else{
      distance_to_home_previous = distance_to_home;
    }
    time_stamp_for_distance = millis();
  }
  distance_to_home = sqrt(pow(x_position, 2) + pow(y_position, 2)); 


  if (distance_to_home > 0){
    speed_demand_RIGHT = 0.8;
    speed_demand_LEFT = 0.8;
  }
  else{
    float distance_coefficient = 100/100.0;
    speed_demand_RIGHT = speed_demand_RIGHT * distance_coefficient;
    speed_demand_LEFT = speed_demand_LEFT * distance_coefficient;
  }

  float output_RIGHT = right_PID.update(speed_demand_RIGHT, velocity_RIGHT);
  float output_LEFT = left_PID.update(speed_demand_LEFT, velocity_LEFT);
  
  if ( sign(output_RIGHT) == true){ digitalWrite( R_DIR_PIN, HIGH );}
  else if ( sign(output_RIGHT) == false){digitalWrite( R_DIR_PIN, LOW );}
  if ( sign(output_LEFT) == true){ digitalWrite( R_DIR_PIN, HIGH );}
  else if ( sign(output_LEFT) == false){digitalWrite( R_DIR_PIN, LOW );}

  // caluclate speed
  elapsed_time = millis() - last_time_check;
  if (elapsed_time > 20){
    last_time_check = millis();
    
    long counts_delta_RIGHT;
    counts_delta_RIGHT = count_right - previous_count_right;
    previous_count_right = count_right;

    long counts_delta_LEFT;
    counts_delta_LEFT = count_left - previous_count_left;
    previous_count_left = count_left;

    velocity_RIGHT = counts_delta_RIGHT / (float)elapsed_time;  // units: [counts/s] 
    velocity_LEFT = counts_delta_LEFT / (float)elapsed_time;  // units: [counts/s] 
  }

  digitalWrite( R_DIR_PIN, LOW );
  digitalWrite( L_DIR_PIN, LOW );
  analogWrite( R_PWM_PIN, -output_RIGHT);
  analogWrite( L_PWM_PIN, -output_LEFT);

    
  if ( distance_to_home_previous < distance_to_home){
    analogWrite(R_PWM_PIN, 0);
    analogWrite(L_PWM_PIN, 0);
    romi_phase = 5;
  } 
}


void go_home(){
  int x_position = kinematics.getX();
  int y_position = kinematics.getY();
  int distance_to_home = sqrt(pow(x_position, 2) + pow(y_position, 2));

  // position PID
  float output = position_PID.update(0, distance_to_home);  // negative output means that we still need to move forward, positive means we moved to far forward



  float pheta_radians = kinematics.getPheta();  


  if (distance_to_home < 100 and distance_to_home > 0 and second_turn_done == false){
     romi_phase = 3;
     second_turn_done = true;
  }

  if (output > 30){
    output = 30;
  }
  else if (output < -30){
    output = -30;
  }


//  if (second_turn_done){
//    if ( x_position > 0){  // -> FORWARD
//      digitalWrite( R_DIR_PIN, LOW );
//      digitalWrite( L_DIR_PIN, LOW );
//      analogWrite( R_PWM_PIN, 14.5 + output);
//      analogWrite( L_PWM_PIN, 14 + output);
//    }
//    else if ( x_position < 0){ // BACKWARD
//      digitalWrite( R_DIR_PIN, HIGH );
//      digitalWrite( L_DIR_PIN, HIGH ); 
//      analogWrite( R_PWM_PIN, 14 - output);
//      analogWrite( L_PWM_PIN, 14 - output); 
//    }
//  }
//  else{
    if ( x_position > 0){  // -> FORWARD
      digitalWrite( R_DIR_PIN, LOW );
      digitalWrite( L_DIR_PIN, LOW );
      analogWrite( R_PWM_PIN, 14 + output - heading_output);
      analogWrite( L_PWM_PIN, 14 + output + heading_output);
    }
    else if ( x_position < 0){ // BACKWARD
      digitalWrite( R_DIR_PIN, HIGH );
      digitalWrite( L_DIR_PIN, HIGH ); 
      analogWrite( R_PWM_PIN, 14 - output);
      analogWrite( L_PWM_PIN, 14 - output);   
    } 
//  }
  
  if ( (x_position < 1) and (x_position > -1)){
    analogWrite(R_PWM_PIN, 0);
    analogWrite(L_PWM_PIN, 0);
    romi_phase = 5;
  } 

}





void turn_home(){
  
  int x_position = kinematics.getX();
  int y_position = kinematics.getY();
  float pheta_radians = kinematics.getPheta();  

  required_angle = 3.14159265 + atan2(y_position, x_position);
  heading_output = heading_PID.update(required_angle, pheta_radians);

  if (required_angle > pheta_radians){  // NEGATIVE required angle -> means need to rotate left 
    digitalWrite( L_DIR_PIN, HIGH );  //LEFT wheel backwards 
    digitalWrite( R_DIR_PIN, LOW );  //RIGHT forwards
    analogWrite( R_PWM_PIN, 18);
    analogWrite( L_PWM_PIN, 18);
  }
  else if (required_angle < pheta_radians){ 
    digitalWrite( L_DIR_PIN, LOW );  //LEFT wheel backwards 
    digitalWrite( R_DIR_PIN, HIGH );  //RIGHT forwards
    analogWrite( R_PWM_PIN, 18);
    analogWrite( L_PWM_PIN, 18);
  }
  if ( (required_angle - pheta_radians) < 0.01 and (required_angle - pheta_radians) > -0.01 ){
    analogWrite(R_PWM_PIN, 0);
    analogWrite(L_PWM_PIN, 0);
    romi_phase = 4;
    left_PID.reset();
    right_PID.reset();
  }    
}





void turn_to_temp_origin(){
  float pheta_radians = kinematics.getPheta();

  if (pheta_radians > temp_pheta){
    digitalWrite(R_DIR_PIN, HIGH);
    analogWrite(R_PWM_PIN, 22);
    digitalWrite(L_DIR_PIN, LOW);
    analogWrite(L_PWM_PIN, 22);
  }
  if (pheta_radians < temp_pheta){
    digitalWrite(R_DIR_PIN, LOW);
    analogWrite(R_PWM_PIN, 22);
    digitalWrite(L_DIR_PIN, HIGH);
    analogWrite(L_PWM_PIN, 22);
  }
  if ( (pheta_radians-temp_pheta) < 0.02 and (pheta_radians-temp_pheta) > -0.02 ){
    analogWrite(R_PWM_PIN, 0);
    analogWrite(L_PWM_PIN, 0);
    romi_phase = 2;
  }
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


  // NOT ON LINE
  if (readings_sum < 350){  // used to be 250 !!

    if (previous_reading_sum > 350){  // used to be 250 !!
      just_lost_the_line = true;
    }
  
    if (just_lost_the_line){
      temp_pheta = kinematics.getPheta();
      
      just_lost_the_line = false;
    }
    
    if (onLine_past) {
      // was on line before 
      // ideally i would like to have a way of storing information 
      // whether romi was on a line within last 2 seconds
      // and if was, on which side of the line he is now

      // then stop motors. 
      analogWrite( R_PWM_PIN, 0);
      analogWrite( L_PWM_PIN, 0);
      
      if (last_High == 1){  // we are on the left of the line
        digitalWrite(L_DIR_PIN, LOW);
        analogWrite( L_PWM_PIN, 18);
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, 18); 

        Serial.println("on the left");
        Serial.print(temp_pheta);
        Serial.print(", ");
        Serial.println(kinematics.getPheta() - temp_pheta);
        
        if (temp_pheta - kinematics.getPheta() > 1.7){  // if no line found within 90 degrees -> TERMINATE - end of line
          analogWrite(R_PWM_PIN, 0);
          analogWrite(L_PWM_PIN, 0);
          romi_phase = 1;   
        }
      }
      else if (last_High == 2){  // we are on the right of the line
        digitalWrite(R_DIR_PIN, LOW);
        analogWrite(R_PWM_PIN, 18);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(L_PWM_PIN, 18);

        Serial.println("on the right");
        Serial.print(temp_pheta);
        Serial.print(", ");
        Serial.println(kinematics.getPheta() - temp_pheta);

        if (kinematics.getPheta() - temp_pheta > 1.7){  // if no line found within 90 degrees -> TERMINATE - end of line
          analogWrite(R_PWM_PIN, 0);
          analogWrite(L_PWM_PIN, 0);
          romi_phase = 1;
        }
      }
      delay(20);
    }
    else { // no line before
      analogWrite(R_PWM_PIN, r_pwm);
      analogWrite(L_PWM_PIN, l_pwm);
    }
  }




  // ON LINE  
  else {    
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(L_DIR_PIN, LOW);
    onLine_past = true;
    float left_probability = (float) left_lineSensor / readings_sum;
    float centre_probability = (float) centre_lineSensor / readings_sum;
    float right_probability = (float) right_lineSensor / readings_sum;

    score_lineSensor = (1000*left_probability) + (2000*centre_probability) + (3000*right_probability) - 2000;

    pid_output_lineSensor = lineSensor_PID.update(0, score_lineSensor);
    
    if (score_lineSensor < 0) {  // we are on the right of the line

      
      analogWrite( L_PWM_PIN , l_pwm - (-pid_output_lineSensor));
      analogWrite( R_PWM_PIN, r_pwm + (-pid_output_lineSensor));


//      digitalWrite(R_DIR_PIN, LOW);
//      analogWrite(R_PWM_PIN, r_pwm + 2*(-pid_output_lineSensor));
//      digitalWrite(L_DIR_PIN, HIGH);
//      analogWrite(L_PWM_PIN, l_pwm + 2*(-pid_output_lineSensor));

      last_High = 2;

      
 
    }
    else{ // if (score_lineSensor >= 0){  // we are on the left of the line


      
      analogWrite( R_PWM_PIN, r_pwm - pid_output_lineSensor);
      analogWrite( L_PWM_PIN , l_pwm + pid_output_lineSensor);


//      digitalWrite(R_DIR_PIN, HIGH);
//      analogWrite(R_PWM_PIN, r_pwm + 2*(-pid_output_lineSensor));
//      digitalWrite(L_DIR_PIN, LOW);
//      analogWrite(L_PWM_PIN, l_pwm + 2*(-pid_output_lineSensor));

      last_High = 1;
    }
  }
}
