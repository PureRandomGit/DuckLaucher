#include "SimpleRSLK.h"

unsigned long lastTime = 0;

bool shot = false;

// Light sensor calibration values
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
uint16_t sensorMinVal[LS_NUM_SENSORS] = {828, 730, 655, 633, 582, 715, 520, 742};

// Robot Constants
const int MAX_SPEED = 100;
const int BASE_SPEED = 90;
const int TURN_SPEED = 100;
const int SHOOTER_PIN = P10_4; // Pin to control the shooter mechanism

// PID constants
const float KP = 0.05;
const float KI = 0.001;
const float KD = 0.01;

const int GOAL = 3500;  // Center position for line sensor

float lastError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

bool bumperPreviouslyPressed = false; // Tracks last bumper state to detect new hits
bool pendingHomeAfterTurn = false;    // Signals that the next turn should go to HOME

enum class State {
    START,
    RESTART,
    PATH,
    SHOOT,
    TURN,
    HOME,
    DONE
};

State state = State::START;

void printState(State s) {
    Serial.println(static_cast<int>(s));
}

void setup()
{
    delay(100);

    Serial.begin(115200);

    setupRSLK();
    clearMinMax(sensorMinVal, sensorMaxVal);

    // reset encoders
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Shooter pin
    pinMode(SHOOTER_PIN, OUTPUT);
    digitalWrite(SHOOTER_PIN, LOW);

    setupWaitBtn(LP_LEFT_BTN); // Left botton on the Launchpad
    setupLed(RED_LED);         // Use red led to signal waiting for button

    /* Initialize motors */
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    setMotorSpeed(BOTH_MOTORS, 0);

    enableMotor(BOTH_MOTORS);
}

void loop()
{
    // Prints state every 100ms (10Hz)
    if ((millis() - lastTime) >= 100)
    {
        // printState(state);
        lastTime = millis();
    }

    switch (state) {
        case State::START:    start();    break;
        case State::RESTART:  restart();  break;
        case State::PATH:     path();     break;
        case State::SHOOT:    shoot();    break;
        case State::TURN:     turn();     break;
        case State::HOME:     home();     break;
        case State::DONE:     done();     break;
    }
}

void start() {
    // Serial.println("Start");
    waitBtnPressed(LP_LEFT_BTN, "\nPush left button on Launchpad to start challenge.\n", RED_LED);
    delay(100); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    bumperPreviouslyPressed = isBumperPressed();
    pendingHomeAfterTurn = false;

    state = State::PATH;
}

void restart() {
    // Serial.println("restart");
    delay(100); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    bumperPreviouslyPressed = isBumperPressed();
    pendingHomeAfterTurn = false;

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

    bool bumperPressed = isBumperPressed();
    bool newBumperPress = bumperPressed && !bumperPreviouslyPressed;
    bumperPreviouslyPressed = bumperPressed;

    // Calculate time delta for proper PID
    unsigned long currentTime = millis();
    float dt = (currentTime - lastPIDTime) / 1000.0;  // Convert to seconds
    if (dt <= 0) { dt = 0.001; }
    lastPIDTime = currentTime;

    // PID calculation
    int error = linePos - GOAL;
    
    // Proportional term
    float P = KP * error;
    
    // Integral term
    integral += error * dt;
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

    if (newBumperPress) {
        setMotorSpeed(BOTH_MOTORS, 0);  // Stop motors
        lastError = 0;
        integral = 0;
        lastPIDTime = millis();
        delay(400);
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

void shoot() {
    delay(100);
    // Serial.println("Shooting duck...");
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(300);
    digitalWrite(SHOOTER_PIN, LOW);
    delay(50);
    shot = true;
    pendingHomeAfterTurn = true;
    state = State::TURN;
}

void turn() {
    static bool turningStarted = false;
    static uint16_t startLeftCount = 0;
    static uint16_t startRightCount = 0;
    
    // Calculate encoder counts needed for 180 degree turn
    const uint16_t COUNTS_FOR_180 = 180; // Calibrated value for 180 degree turn
    
    if (!turningStarted) {
        // Start the turn
        startLeftCount = getEncoderLeftCnt();
        startRightCount = getEncoderRightCnt();
        
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
        setMotorSpeed(BOTH_MOTORS, TURN_SPEED);
        
        turningStarted = true;
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
        
        // Serial.println("Turn complete!");
        
        // Reset encoders and PID for next path
        resetLeftEncoderCnt();
        resetRightEncoderCnt();

        if (pendingHomeAfterTurn) {
            pendingHomeAfterTurn = false;
            state = State::HOME;
        } else {
            state = State::PATH;
        }
    }
}

void home() {
    // Drive straight without gyro until bumper is pressed
    static bool homeStarted = false;
    
    if (!homeStarted) {
        // Initialize for driving straight
        resetLeftEncoderCnt();
        resetRightEncoderCnt();
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
        setMotorSpeed(BOTH_MOTORS, BASE_SPEED);
        homeStarted = true;
        bumperPreviouslyPressed = isBumperPressed();
    }
    
    // Check for bumper press
    bool bumperPressed = isBumperPressed();
    bool newBumperPress = bumperPressed && !bumperPreviouslyPressed;
    bumperPreviouslyPressed = bumperPressed;
    
    if (newBumperPress) {
        // Stop motors
        setMotorSpeed(BOTH_MOTORS, 0);
        homeStarted = false;
        delay(50);
        state = State::DONE;
    }
}

void done() {
    setMotorSpeed(BOTH_MOTORS, 0);
    // Serial.println("DONE - Press bumper to restart");
    
    // Wait for bumper press to restart
    bool bumperPressed = isBumperPressed();
    bool newBumperPress = bumperPressed && !bumperPreviouslyPressed;
    bumperPreviouslyPressed = bumperPressed;
    
    if (newBumperPress) {
        shot = false;
        state = State::RESTART;
    }
}

// Helper functions
boolean isBumperPressed() {
    return (isBumpSwitchPressed(2) && isBumpSwitchPressed(3)); // TODO: adjust switch numbers as needed
}

boolean isButtonPressed() {
    return (digitalRead(LP_LEFT_BTN) == 0); // TODO: Change button
}
