#ifndef _Kinematics
#define _Kinematics_h

//You may want to use some/all of these variables
//const float WHEEL_DIAMETER    = ??;
//const float WHEEL_RADIUS      = ??;
//const float WHEEL_SEPERATION  = ??;
//const float GEAR_RATIO        = ??;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
//const float COUNTS_PER_WHEEL_REVOLUTION =  ??;
//const float COUNTS_PER_MM               = ??;

class Kinematics
{
  public:
    
    Kinematics();   // Constructor, required.

    // Write your method functions:
    // ...
    void update(int right_encoder, int left_encoder);
    void update_ICC(int right_encoder, int left_encoder);
    float getX();
    float getY();
    float getPheta();
    int getPhetaDegrees();
    void setX(float x);
    void setY(float y);
    void setPheta(float phe);
    
    
  private:
    
    //Private variables and methods go here
    float x_cord;
    float y_cord;
    float pheta;
    int pheta_degrees;

    int right_encoder_last;
    int left_encoder_last;
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
  x_cord = 0;
  y_cord = 0;
  pheta = 0;
  pheta_degrees = 0;

  right_encoder_last = 0;
  left_encoder_last = 0;
  
}

void Kinematics::update(int right_encoder, int left_encoder) {
  
  float right_en_difference = right_encoder - right_encoder_last;  // units: counts
  float left_en_difference = left_encoder - left_encoder_last;  // units: counts
  
  // convert from counts to mm
  right_en_difference = right_en_difference * ((3.14159265 * 70)/1440);  // units: mm 
  left_en_difference = left_en_difference * ((3.14159265 * 70)/1440);  // units: mm 
  float d = (right_en_difference + left_en_difference)/2;

  pheta = pheta + (left_en_difference - right_en_difference)/140;
  pheta_degrees = pheta * (360/(2 * 3.14159265));  // convert: radians -> degrees 
  
  x_cord = x_cord + d*cos(pheta);
  y_cord = y_cord + d*sin(pheta);
  
  right_encoder_last = right_encoder;
  left_encoder_last = left_encoder;
}


void Kinematics::update_ICC(int right_encoder, int left_encoder) {
  
  float n_r = right_encoder - right_encoder_last;  // units: counts
  float n_l = left_encoder - left_encoder_last;  // units: counts

  // convert from counts to mm
  n_r = n_r * ((3.14159265 * 70)/1440);  // units: mm 
  n_l = n_l * ((3.14159265 * 70)/1440);  // units: mm

  if (n_r == n_l){  // MOVING STRAIGHT
    float d = (n_r + n_l)/2;
    x_cord = x_cord + d*cos(pheta);
    y_cord = y_cord + d*sin(pheta);
  }
  
  else{  // TURNING  
    float delta_angle = (n_r - n_l)/140;
    float radius_ICC = (140/2) * ( (n_l + n_r) / (n_r - n_l) ); 
    float x_ICC = x_cord - radius_ICC*sin(pheta);
    float y_ICC = y_cord + radius_ICC*cos(pheta);
  
    pheta = pheta + delta_angle;
    pheta_degrees = pheta * (360/(2 * 3.14159265));  // convert: radians -> degrees 
    
    x_cord = cos(delta_angle)*(x_cord - x_ICC) - sin(delta_angle)*(y_cord - y_ICC) + x_ICC;
    y_cord = sin(delta_angle)*(x_cord - x_ICC) + cos(delta_angle)*(y_cord - y_ICC) + y_ICC;
  }

  
  right_encoder_last = right_encoder;
  left_encoder_last = left_encoder;
}


float Kinematics::getX(){
  return x_cord;
}

float Kinematics::getY(){
  return y_cord;
}

float Kinematics::getPheta(){
  return pheta;
}

int Kinematics::getPhetaDegrees(){
  return pheta_degrees;
}

void Kinematics::setX(float x){
  x_cord = x; 
}

void Kinematics::setY(float y){
  y_cord = y; 
}

void Kinematics::setPheta(float phe){
  pheta = phe; 
}



#endif
