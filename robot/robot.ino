
#include "SimpleRSLK.h"
#include "BNO055_support.h" //Contains the bridge code between the API and Arduino
#include <Wire.h>

struct bno055_t myBNO;
struct bno055_euler myEulerData; // Structure to hold the Euler data

float wheelDiameter = 2.5;  // Diameter of Romi wheels in inches
int cntPerRevolution = 360; // Number of encoder (rising) pulses every time the wheel turns completely

float initialHeading;

float crashedCount;
int currentCount;
unsigned long lastTime = 0;
String btnMsg = " ";

// Light sensor calibration values
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
uint16_t sensorMinVal[LS_NUM_SENSORS] = {756, 753, 630, 608, 513, 654, 634, 696};

// Robot Constants
const int MAX_SPEED = 100; // TODO: Find max speed
const int BASE_SPEED = 100; // TODO: Find base speed
const int TURN_SPEED = 100; // TODO: Find max turn speed
const int GOAL = 3500;  // Center position for line sensor
const int SHOOTER_PIN = P10_4; // Pin to control the shooter mechanism

// PID constants - tune these!
const float KP = 0.06;    // Proportional gain
const float KI = 0.005;  // Integral gain (start small)
const float KD = 0.025;     // Derivative gain

// PID state variables
float lastError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

// Alignment Constants
const float HEADING_TOLERANCE = 1;  // degrees // TODO: adjust as needed
const unsigned long TIMEOUT = 3000;   // .5 second timeout // TODO: adjust as needed
const int PUSH_SPEED = 30; // TODO: adjust as needed

enum class State {
    START,
    PATH,
    ALIGN,
    SHOOT,
    TURN,
    DONE
};

State state = State::START;

void printState(State s) {
    Serial.println(static_cast<int>(s));
}

void setup()
{
    delay(100);

    // Initialize I2C communication
    Wire.begin();

    // Initialization of the BNO055
    BNO_Init(&myBNO); // Assigning the structure to hold information about the device

    // Configuration to NDoF mode
    bno055_set_operation_mode(OPERATION_MODE_NDOF);

    delay(1); // Wait for gyro to settle
    Serial.begin(115200);

    setupRSLK();
    clearMinMax(sensorMinVal, sensorMaxVal);

    // reset encoders
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Shooter pin
    pinMode(SHOOTER_PIN, OUTPUT);
    digitalWrite(SHOOTER_PIN, LOW);

    // PID
    lastPIDTime = millis();

    setupWaitBtn(LP_LEFT_BTN); // Left botton on the Launchpad
    setupLed(RED_LED);         // Use red led to signal waiting for button

    /* Initialize motors */
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, 0);

    // Read initial heading
    delay(1000);
    bno055_read_euler_hrp(&myEulerData); // Update Euler data into the structure
    initialHeading = float(myEulerData.h) / 16.00;
    Serial.print("Initial Heading(Yaw): "); // To read out the Heading (Yaw)
    Serial.println(initialHeading);

    enableMotor(BOTH_MOTORS);
}

void loop()
{
    delay(5); // TODO: See of this can be removed

    // Prints state every 100ms (10Hz)
    if ((millis() - lastTime) >= 100)
    {
        printState(state);
        lastTime = millis();
    }

    switch (state) {
        case State::START:    start();    break;
        case State::PATH:     path();     break;
        case State::ALIGN:    align();    break;
        case State::SHOOT:    shoot();    break;
        case State::TURN:     turn();     break;
        case State::DONE:     done();     break;
    }
}

void start() {
    Serial.println("Start");
    waitBtnPressed(LP_LEFT_BTN, "\nPush left button on Launchpad to start challenge.\n", RED_LED);
    delay(500); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
    bno055_read_euler_hrp(&myEulerData);
    initialHeading = float(myEulerData.h) / 16.00;

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    // Reset PID state
    lastError = 0;
    integral = 0;
    lastPIDTime = millis();

    state = State::PATH;
}

void path() {
  /* Valid values are either:
   *  DARK_LINE  if your floor is lighter than your line
   *  LIGHT_LINE if your floor is darker than your line
   */
  uint8_t lineColor = DARK_LINE;

  readLineSensor(sensorVal);

    /*
    * Take current sensor values and adjust using previous calibration values
    * Output: sensorCalVal
    */
    readCalLineSensor(
        sensorVal,
        sensorCalVal,
        sensorMinVal,
        sensorMaxVal,
        lineColor
    );

    uint32_t linePos = getLinePosition(sensorCalVal,lineColor);

    // Calculate time delta for proper PID
    unsigned long currentTime = millis();
    float dt = (currentTime - lastPIDTime) / 1000.0;  // Convert to seconds
    if (dt <= 0) { dt = 0.001; }
    lastPIDTime = currentTime;

    // PID calculation
    int error = linePos - GOAL;
    
    // Proportional term
    float P = KP * error;
    
    // Integral term (accumulated error over time)
    integral += error * dt;
    // Anti-windup: limit integral to prevent it from growing too large
    integral = constrain(integral, -1000, 1000);
    float I = KI * integral;
    
    // Derivative term (rate of change of error)
    float derivative = (error - lastError) / dt;
    float D = KD * derivative;
    
    // Total PID output
    float motor_speed_delta = P + I + D;
    
    // Update last error for next iteration
    lastError = error;

    // Apply PID correction to base speed
    int left_motor_speed = constrain(BASE_SPEED + motor_speed_delta, 0, MAX_SPEED);
    int right_motor_speed = constrain(BASE_SPEED - motor_speed_delta, 0, MAX_SPEED);

    setMotorSpeed(LEFT_MOTOR, left_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, right_motor_speed);

    // Output PID data for Serial Plotter (format: label:value label:value)
    Serial.print("Setpoint:");
    Serial.print(GOAL);
    Serial.print(",");
    Serial.print("Current:");
    Serial.print(linePos);
    Serial.print(",");
//    Serial.print("Error:");
//    Serial.print(error);
//    Serial.print(",");
//    Serial.print("P:");
//    Serial.print(P);
//    Serial.print(",");
//    Serial.print("I:");
//    Serial.print(I);
//    Serial.print(",");
//    Serial.print("D:");
//    Serial.print(D);
//    Serial.print(",");
    Serial.print("Output:");
    Serial.println(motor_speed_delta);

    if (isBumperPressed()) {
        setMotorSpeed(BOTH_MOTORS, 0);  // Stop motors
        lastError = 0;
        integral = 0;
        lastPIDTime = millis();
        state = State::SHOOT;
    }
    else if (isButtonPressed()) {
        setMotorSpeed(BOTH_MOTORS, 0);  // Stop motors
        lastError = 0;
        integral = 0;
        lastPIDTime = millis();
        state = State::DONE;
    }
}

void align() {
    static unsigned long alignStartTime = 0;
    static bool aligning = false;
    static float lastHeading = 0;
        
    if (!aligning) {
        alignStartTime = millis();
        bno055_read_euler_hrp(&myEulerData);
        lastHeading = float(myEulerData.h) / 16.00;
        aligning = true;
        
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, PUSH_SPEED);
        Serial.println("Aligning against wall...");
    }
    
    bno055_read_euler_hrp(&myEulerData);
    float currentHeading = float(myEulerData.h) / 16.00;
    float headingChange = abs(currentHeading - lastHeading);
        Serial.print(",");
    Serial.print("Angle:");
    Serial.print(currentHeading);
    Serial.print(",");
    
    if (headingChange < HEADING_TOLERANCE || (millis() - alignStartTime) >= TIMEOUT) {
        setMotorSpeed(BOTH_MOTORS, 0);
        aligning = false;
        
        if (headingChange < HEADING_TOLERANCE) {
            Serial.println("Aligned!");
        } else {
            Serial.println("Alignment timeout");
        }
        
        state = State::SHOOT;
    }
    
    lastHeading = currentHeading;
}

void shoot() {
    delay(500); // TODO: Adjust shoot time as needed
    Serial.println("Shooting duck...");
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(300); // TODO: Adjust shoot time as needed
    digitalWrite(SHOOTER_PIN, LOW);
    delay(200); // TODO: Adjust shoot time as needed

    state = State::TURN;
}

void turn() {
    static bool turningStarted = false;
    static uint16_t startLeftCount = 0;
    static uint16_t startRightCount = 0;
    
    // Calculate encoder counts needed for 180 degree turn
    // Adjust the 500 value based on your robot's dimensions
    const uint16_t COUNTS_FOR_180 = 300; // TODO: Calibrate this value
    
    if (!turningStarted) {
        // Start the turn
        startLeftCount = getEncoderLeftCnt();
        startRightCount = getEncoderRightCnt();
        
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, TURN_SPEED);
        
        turningStarted = true;
        Serial.println("Starting 180 turn");
    }
    
    // Check if we've turned enough
    uint16_t leftTurned = getEncoderLeftCnt() - startLeftCount;
    uint16_t rightTurned = getEncoderRightCnt() - startRightCount;
    uint16_t avgTurned = (leftTurned + rightTurned) / 2;
    
    if (avgTurned >= COUNTS_FOR_180) {
        // Turn complete
        setMotorSpeed(BOTH_MOTORS, 0);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        turningStarted = false;
        
        Serial.println("Turn complete!");
        
        // Reset encoders and PID for next path
        resetLeftEncoderCnt();
        resetRightEncoderCnt();
        lastError = 0;
        integral = 0;
        lastPIDTime = millis();
        
        state = State::PATH;
    }
}

void done() {
    setMotorSpeed(BOTH_MOTORS, 0);
    disableMotor(BOTH_MOTORS);
    Serial.println("DONE - Press button to restart");
    delay(1000);
    if (isButtonPressed()) {
        state = State::START;
    }
}

// Helper functions
float calculateAngleDifference(float target, float current) {
    float delta = fmod((target - current + 360.0), 360.0);
    if (delta > 180.0) {
        delta = delta - 360.0;
    }
    return delta;
}

boolean isBumperPressed() {
    return (isBumpSwitchPressed(2) && isBumpSwitchPressed(3)); // TODO: adjust switch numbers as needed
}

boolean isButtonPressed() {
    return (digitalRead(LP_LEFT_BTN) == 0); // TODO: Change button
}
