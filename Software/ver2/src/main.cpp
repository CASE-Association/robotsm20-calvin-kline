#include <Arduino.h>
#include <Sparkfun_TB6612.h>
#include <QTRSensors.h>

/******************************************************************************
TestRun.ino
TB6612FNG H-Bridge Motor Driver Example code
Michelle @ SparkFun Electronics
8/20/16
https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library

Uses 2 motors to show examples of the functions in the library.  This causes
a robot to do a little 'jig'.  Each movement has an equal and opposite movement
so assuming your motors are balanced the bot should end up at the same place it
started.

Resources:
TB6612 SparkFun Library

Development environment specifics:
Developed on Arduino 1.6.4
Developed with ROB-9457
******************************************************************************/

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A4, A2, A3, A5, A1, 12, 11, 10}, SensorCount); //D5 - D12

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 256; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

}

int baseSpeed = 255;
int lastError = 0;
int Kp = 13;
int Kd = 13;

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;


  int change = (int) error/Kp + Kd*(error-lastError);
  //Kd * (error - lastError); 
  lastError = error;

  int rightMotorSpeed = baseSpeed - change;
  int leftMotorSpeed = baseSpeed + change;

  if (rightMotorSpeed > baseSpeed ) rightMotorSpeed = baseSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > baseSpeed ) leftMotorSpeed = baseSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;    
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

  
  motorR.drive(rightMotorSpeed);
  motorL.drive(leftMotorSpeed);


  /*
  Serial.print("Error:");
  Serial.println(error);

  Serial.print("Change:");
  Serial.println(change);

  Serial.print("Right:");
  Serial.println(rightMotorSpeed);

  Serial.print("Left:");
  Serial.println(leftMotorSpeed);
  Serial.println("");
  delay(500);
  */
  
  
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  /*
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  
  Serial.println(position);

 */



 


  

  //forward(motor1,motor2,255);

   /*
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor1.drive(255,1000);
   motor1.drive(-255,1000);
   motor1.brake();
   delay(1000);
   
   //Use of the drive function which takes as arguements the speed
   //and optional duration.  A negative speed will cause it to go
   //backwards.  Speed can be from -255 to 255.  Also use of the 
   //brake function which takes no arguements.
   motor2.drive(255,1000);
   motor2.drive(-255,1000);
   motor2.brake();
   delay(1000);
   
   //Use of the forward function, which takes as arguements two motors
   //and optionally a speed.  If a negative number is used for speed
   //it will go backwards
   forward(motor1, motor2, 150);
   delay(1000);
   
   //Use of the back function, which takes as arguments two motors 
   //and optionally a speed.  Either a positive number or a negative
   //number for speed will cause it to go backwards
   back(motor1, motor2, -150);
   delay(1000);
   
   //Use of the brake function which takes as arguments two motors.
   //Note that functions do not stop motors on their own.
   brake(motor1, motor2);
   delay(1000);
   
   //Use of the left and right functions which take as arguements two
   //motors and a speed.  This function turns both motors to move in 
   //the appropriate direction.  For turning a single motor use drive.
   left(motor1, motor2, 100);
   delay(1000);
   right(motor1, motor2, 100);
   delay(1000);
   
   //Use of brake again.
   brake(motor1, motor2);
   delay(1000);
   */
}