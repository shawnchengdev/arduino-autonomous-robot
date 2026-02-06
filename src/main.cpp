#include <Pixy2.h>
#include <Servo.h>
#include <CytronMotorDriver.h>

// All pin assignments
CytronMD left_Motor(PWM_DIR, 1, 2);
CytronMD right_Motor(PWM_DIR, 3, 4);

const int LEFT_IR_SENSOR = 6;
const int RIGHT_IR_SENSOR = 5;

// Variable initialization
int mode = 1;

// Function declarations
int line_following();


void setup() {
  pinMode(LEFT_IR_SENSOR, INPUT);
  pinMode(RIGHT_IR_SENSOR, INPUT);
  
  Serial.begin(9600);
  Serial.println("Starting Program...");
}


void loop() {
  if (mode == 1) 
  {
    line_following();
  }
}


// Functions definitions
int line_following() {
  
}