#include "SimpleRSLK.h"
#include "BNO055_support.h"   //Contains the bridge code between the API and Arduino
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

float wheelDiameter = 2.5;      // Diameter of Romi wheels in inches
int cntPerRevolution = 360;   // Number of encoder (rising) pulses every time the wheel turns completely

float initialHeading;

#define BASE_SPEED 50         // Default speed of the robot
#define TURN_SPEED 30         // Default turn speed
#define DESIRED_HEADING1 -60     // Heading to object1 (figure it out)
#define RETURN_HEADING1 120      // Heading to return on (figure it out)
#define DESIRED_HEADING2 0     // Heading to object2 (figure it out)
#define RETURN_HEADING2 180      // Heading to return on (figure it out)
#define DESIRED_HEADING3 60     // Heading to object3 (figure it out)
#define RETURN_HEADING3 -120      // Heading to return on (figure it out)

float P = 1.0;
unsigned long lastTime = 0;
typedef enum State
{
  START,
  TURN_TO_TARGET1,
  GO_BUMP1,
  TURN_TO_HOME1,
  GO_HOME1,
  TURN_TO_TARGET2,
  GO_BUMP2,
  TURN_TO_HOME2,
  GO_HOME2,
  TURN_TO_TARGET3,
  GO_BUMP3,
  TURN_TO_HOME3,
  GO_HOME3,
  DONE,
  SLEEP
} State;

State state = START;     // Initial state

void setup()
{
  // Initialize I2C communication
  Wire.begin();

  // Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  // Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);       // Wait for gyro to settle
  Serial.begin(115200);

  setupRSLK();

  // reset encoders
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  setupWaitBtn(LP_LEFT_BTN);    // Left botton on the Launchpad
  setupLed(RED_LED);            // Use red led to signal waiting for button

  /* Initialize motors */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS,0);

  // Read initial heading
  delay(1000);
  bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure
  initialHeading = float(myEulerData.h) / 16.00;
  Serial.print("Initial Heading(Yaw): ");       //To read out the Heading (Yaw)
  Serial.println(initialHeading);

  enableMotor(BOTH_MOTORS);
}

// MAIN LOOP // 

void loop()
{
  delay(50);      // Don't loop too fast

  if ((millis() - lastTime) >= 10) // Print state at 10hz
  {
    Serial.println(state);
    lastTime = millis();
  }

  // Run Current State
  switch (state) {

    case START: 
      Serial.println("Start");
      /* Wait until button is pressed to start robot */
      waitBtnPressed(LP_LEFT_BTN,"\nPush left button on Launchpad to start challenge.\n",RED_LED);
      bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure
      initialHeading = float(myEulerData.h) / 16.00;
      state = TURN_TO_TARGET1;
    break;
    
    // START TARGET 1 CODE //

    // Turns to target 1 heading
    case TURN_TO_TARGET1:
      if (turnTo(DESIRED_HEADING1,TURN_SPEED)) {
        state = GO_BUMP1;
      }
    break;

    // Drives forward indefinitely until a target is hit...
   case GO_BUMP1:
    	/* Keep checking if the robot has hit an object */
  	  if (driveToDistanceHeading(DESIRED_HEADING1,15)) {
        state = TURN_TO_HOME1;
      };
  	break;

    // Turn back to home (target heading +/- 180)
    case TURN_TO_HOME1:
      if (turnTo(RETURN_HEADING1,TURN_SPEED)) {
        state = GO_HOME1;
      }
    break;

    // Drive home...
    case GO_HOME1:
      if (driveToDistanceHeading(RETURN_HEADING1,15)) {
        state = TURN_TO_TARGET2;
      };
  	break;

    case TURN_TO_TARGET2:
      if (turnTo(DESIRED_HEADING2,TURN_SPEED)) {
        state = GO_BUMP2;
      }
    break;

    // Drives forward indefinitely until a target is hit...
   case GO_BUMP2:
    	/* Keep checking if the robot has hit an object */
  	  if (driveToDistanceHeading(DESIRED_HEADING2,15)) {
        state = TURN_TO_HOME2;
      };
  	break;

    // Turn back to home (target heading +/- 180)
    case TURN_TO_HOME2:
      if (turnTo(RETURN_HEADING2,TURN_SPEED)) {
        state = GO_HOME2;
      }
    break;

    // Drive home...
    case GO_HOME2:
      if (driveToDistanceHeading(RETURN_HEADING2,15)) {
        state = TURN_TO_TARGET3;
      };
  	break;

    case TURN_TO_TARGET3:
      if (turnTo(DESIRED_HEADING3,TURN_SPEED)) {
        state = GO_BUMP3;
      }
    break;

    // Drives forward indefinitely until a target is hit...
   case GO_BUMP3:
    	/* Keep checking if the robot has hit an object */
  	  if (driveToDistanceHeading(DESIRED_HEADING3,15)) {
        state = TURN_TO_HOME3;
      };
  	break;

    // Turn back to home (target heading +/- 180)
    case TURN_TO_HOME3:
      if (turnTo(RETURN_HEADING3,TURN_SPEED)) {
        state = GO_HOME3;
      }
    break;

    // Drive home...
    case GO_HOME3:
      if (driveToDistanceHeading(RETURN_HEADING3,15)) {
        state = DONE;
      };
  	break;
    // END TARGET 3 CODE //

    case DONE:  /* Halt motors */
      Serial.println("DONE");
      disableMotor(BOTH_MOTORS);
      state = SLEEP;
    break;

    case SLEEP:
    break;

  } // end state
}

boolean crashed()
{
 return (isBumpSwitchPressed(2) || isBumpSwitchPressed(3));
}

/*
 * Drive distance
 */
boolean driveToDistanceHeading(int desiredHeading, int speed) {
  int headingError = calculateDifferenceBetweenAngles(desiredHeading, getCurrentRealtiveHeadingToStart());
  int adjustSpeed = headingError * P;
  adjustSpeed = constrain(adjustSpeed,-10,10);

  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorSpeed(LEFT_MOTOR, constrain(speed + adjustSpeed, 0, 100));
  setMotorSpeed(RIGHT_MOTOR, constrain(speed - adjustSpeed, 0, 100));

    for(int x = 0;x<TOTAL_BP_SW;x++) {
      		/* Check if bump switch was pressed */
      			if(isBumpSwitchPressed(x) == true) {
              setMotorSpeed(BOTH_MOTORS, 0);
              setMotorSpeed(RIGHT_MOTOR, 0);
              return true;
      			}
    		  }      
  return false;
}

/*
 * Turn degrees
 */
boolean turnTo(int degrees, int speed) {
  
  int headingError = calculateDifferenceBetweenAngles(degrees, getCurrentRealtiveHeadingToStart());

  if (abs(headingError) < 2) {
     setMotorSpeed(BOTH_MOTORS,0);  // Halt motors
      return true;
  }

  Serial.print("Turn: Heading Error: ");        //To read out the Heading (Yaw)
  Serial.println(headingError);   

  if (headingError >= 0) {
      Serial.println("TURN Right");
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
  } else {
      setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);   
      setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);
      setMotorSpeed(BOTH_MOTORS,speed);
  }

  return false;
}

int getCurrentRealtiveHeadingToStart(){
  bno055_read_euler_hrp(&myEulerData);      //Update Euler data into the structure
  float difference = (float(myEulerData.h) / 16.00) - initialHeading;
  return ((int)difference + 360) % 360;
} 

/*
 * calculateDifferenceBetweenAngles
 * ---------------------------------
 * Return the difference between two angles in a 0-360 system
 * - returns +-179
 */
int calculateDifferenceBetweenAngles(int angle1, int angle2) {
   int delta;

    delta = (angle1 - angle2 + 360) % 360;
       if (delta > 180) delta = delta - 360;

     return delta;
}

