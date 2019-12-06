#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor
{
  public:

    // Required function.
    LineSensor(int pin);   //Constructor

    // Suggested functions.
    void calibrate();       //Calibrate
    int read_raw();         //Return the uncalibrated value from the sensor
    int read_calibrated();  //Return the calibrated value from the sensor

    // You may wish to add other functions!
    // ...
    
  private:
  
    int pin;
    int white_reading_mean;
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::read_raw()
{
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate()
{
  for( int count = 0; count < 50; count++ ) {
    int sensor_reading = analogRead(pin);
    white_reading_mean += sensor_reading;
    delay(10);
  }
  white_reading_mean = white_reading_mean / 50;
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::read_calibrated()
{
   return analogRead(pin)-white_reading_mean;
}


#endif
