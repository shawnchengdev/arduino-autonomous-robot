// Import libraries
#include <Arduino.h>
#include <Pixy2.h>
#include <Servo.h>
#include <CytronMotorDriver.h>


// Pixy2 and servo object initializations
Pixy2 pixy;
Servo claw;

// Pin assignments
CytronMD left_motor(PWM_DIR, 1, 2);
CytronMD right_motor(PWM_DIR, 3, 4);

const int LEFT2_IR_SENSOR = 5;
const int LEFT1_IR_SENSOR = 6;
const int CENTER_IR_SENSOR = 7;
const int RIGHT1_IR_SENSOR = 8;
const int RIGHT2_IR_SENSOR = 9;

// Motor Constants
const int MAX_SPEED = 255;    // Maximum motor speed
const int MIN_SPEED = 50;     // Minimum motor speed (ensures the robot moves instead of stalling)
const int FORWARD_SPEED = 255; // Speed for moving forward when using computer vision to approach the object

// Computer vision constants
const int CENTER_X = 150;     // The x-position that represents the center of the camera's view
const int TARGET_WIDTH = 200; // The object's width when it is close enough to grab
const float KP = 1.5;         // Proportional gain for turning control (used for feedback)
const int TURN_THRESHOLD = 5; // How close the object must be to the center before moving forward

// Variable initialization
int current_mode = 1; // Which stage in competition the rover is in
int previous_turn_direction = 0; // #TODO Implement this track the last turn direction to find the line

// Structures declarations
struct IRSensorState
{
  String active_sensors;
  int motor_speed;
};

struct IRValues // A collection of bools representing all the sensors output
{
  bool left2;
  bool left1;
  bool center;
  bool right1;
  bool right2;
};

struct MotorSpeeds
{
  int left_motor;
  int right_motor;
};

// Lists half of the sensors output, and what speed should be returned
// Only lists half since the output could be mirrored to show represent either side
// The order: Center sensor, Inner sensor, Outer sensor
const IRSensorState IR_SENSOR_STATES[8] = 
{
  {"000", MAX_SPEED},
  {"001", MIN_SPEED},
  {"011", 75},
  {"010", 100},
  {"101", 150},
  {"111", 175},
  {"110", 200},
  {"100", MAX_SPEED}
};

// Function declarations
void line_following();
IRValues get_ir_values();
void stop_motors();
void find_line();
void run_motors(const MotorSpeeds&);
MotorSpeeds get_motor_speeds(const IRValues&);
int search_motor_speed(bool, bool, bool);


void setup() 
{
  // Setting pin modes
  pinMode(LEFT2_IR_SENSOR, INPUT);
  pinMode(LEFT1_IR_SENSOR, INPUT);
  pinMode(CENTER_IR_SENSOR, INPUT);
  pinMode(RIGHT1_IR_SENSOR, INPUT);
  pinMode(RIGHT2_IR_SENSOR, INPUT);
  
  Serial.begin(9600);
  claw.attach(5);  // Attach the servo motor to digital pin 5
  claw.write(0);   // Start with the claw open
  pixy.init();     // Initialize Pixy2 camera
  Serial.println("Starting Program...");
}


void loop() 
{
  if (current_mode == 0)
  {
    // Off
  }
  else if (current_mode == 1) 
  {
    line_following();
  } 
  else if (current_mode == 2)
  {
    // Pixy cam tracking
  } 
  else if (current_mode == 3)
  {
    // Grabbing and lifting can
  } 
}

// Compares current sensor outputs with the IR_SENSOR_STATES to find the proper speed for one motor
int search_motor_speed(bool center_value, bool inner_value, bool outer_value)
{
  String map_key = String(center_value) + String(inner_value) + String(outer_value);
  for (int i = 0; i < sizeof(IR_SENSOR_STATES); i++)
  {
    if (map_key == IR_SENSOR_STATES[i].active_sensors)
    {
      return IR_SENSOR_STATES[i].motor_speed;
    }
  }
  return 0;
}

// Gets the both motors speed by calling search_motor_speed
MotorSpeeds get_motor_speeds(const IRValues& ir_values)
{
  MotorSpeeds motor_speeds;
  motor_speeds.left_motor = search_motor_speed(ir_values.center, ir_values.left1, ir_values.left2);
  motor_speeds.right_motor = -search_motor_speed(ir_values.center, ir_values.right1, ir_values.right2);
  return motor_speeds;
}

// Runs both motors in the directions specified by the parameter motor_speed
void run_motors(const MotorSpeeds& motor_speeds)
{
  left_motor.setSpeed(motor_speeds.left_motor);
  right_motor.setSpeed(motor_speeds.right_motor);

  // #TODO: Track previous_turn_direction here
}

// Runs both motors to make the rover spin depending on the previous_turn_direction
void find_line()
{
  if (previous_turn_direction > 0)
  {
    left_motor.setSpeed(MIN_SPEED);
    right_motor.setSpeed(-MAX_SPEED);
  }
  else
  {
    left_motor.setSpeed(MAX_SPEED);
    right_motor.setSpeed(-MIN_SPEED);
  }
}

// Stops the motors
void stop_motors()
{
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
}

// Gets all the IR sensor data
IRValues get_ir_values()
{
  IRValues ir_values;
  ir_values.left2 = digitalRead(LEFT2_IR_SENSOR);
  ir_values.left1 = digitalRead(LEFT1_IR_SENSOR);
  ir_values.center = digitalRead(CENTER_IR_SENSOR);
  ir_values.right1 = digitalRead(RIGHT1_IR_SENSOR);
  ir_values.right2 = digitalRead(RIGHT2_IR_SENSOR);
  return ir_values;
}

// Makes the rover follow a line through IR sensor data.
void line_following() 
{
  IRValues ir_values = get_ir_values();

  // The rover stops when the center, left inner and right inner sensors all detect the line.
  if (ir_values.center && ir_values.left1 && ir_values.right1)
  {
    stop_motors();
  }
  // The rover goes straight forward when there is conflicting data on where the line is going
  else if ((ir_values.left2 || ir_values.left1) && (ir_values.right1 || ir_values.right2))
  {
    MotorSpeeds motor_speeds = {0, 0};
    run_motors(motor_speeds);
  }
  // The rover goes forward with the possibility of turning when the one of the sensors detects the line.
  else if (ir_values.center || ir_values.left1 || ir_values.right1 || ir_values.left2 || ir_values.right2)
  {
    MotorSpeeds motor_speeds = get_motor_speeds(ir_values);
    run_motors(motor_speeds);
  }
  // The rover spins in a circle if none of the sensors detect the line
  else
  {
    find_line();
  }
}