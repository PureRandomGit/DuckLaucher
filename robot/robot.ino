#include "SimpleRSLK.h"

unsigned long lastTime = 0;

bool shot = false;

// Light sensor calibration values
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};
uint16_t sensorMinVal[LS_NUM_SENSORS] = {537, 628, 491, 536, 500, 551, 459, 624};

// Robot Constants
const int MAX_SPEED = 100;
const int BASE_SPEED = 90;
const int SLOW_SPEED = 40;
const int TURN_SPEED = 100;
const int SHOOTER_PIN = P10_4; 
const uint16_t SLOWDOWN_ENCODER_TICKS = 2000; // 2600

// PID constants
const float KP = 0.05f;
const float KI = 0.0012f;
const float KD = 0.01f;
const float INTEGRAL_LIMIT = 10000.0f; // Windup protection
const float GOAL = 3500.0f;  // Center position for line sensor

float lastError = 0.0f;
float integral = 0.0f;
unsigned long lastPIDTime = 0;

bool bumperPreviouslyPressed = false; // Tracks last bumper state to detect new hits
bool pendingDoneAfterTurn = false;    // Signals that the next turn should end in DONE
uint16_t pathStartEncoderLeft = 0;    // Encoder count when path following started
uint16_t pathStartEncoderRight = 0;   // Encoder count when path following started

enum class State {
    START,
    RESTART,
    PATH,
    SHOOT,
    TURN,
    DONE
};

State state = State::START;

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

    // Initialize PID timer
    lastPIDTime = millis();
    
    enableMotor(BOTH_MOTORS);
}

void loop()
{
    switch (state) {
        case State::START:    start();    break;
        case State::RESTART:  restart();    break;
        case State::PATH:     path();     break;
        case State::SHOOT:    shoot();    break;
        case State::TURN:     turn();     break;
        case State::DONE:     done();     break;
    }
}

void start() {
    waitBtnPressed(LP_LEFT_BTN, "\nPush left button on Launchpad to start challenge.\n", RED_LED);
    delay(100); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    // Reset PID variables on start
    lastError = 0;
    integral = 0;
    lastPIDTime = millis();

    bumperPreviouslyPressed = isBumperPressed();
    pendingDoneAfterTurn = false;
    pathStartEncoderLeft = getEncoderLeftCnt();   // Record encoder count when path starts
    pathStartEncoderRight = getEncoderRightCnt(); // Record encoder count when path starts

    state = State::PATH;
}

void restart() {
    delay(50); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    // Reset PID variables on restart
    lastError = 0;
    integral = 0;
    lastPIDTime = millis();

    bumperPreviouslyPressed = isBumperPressed();
    pendingDoneAfterTurn = false;
    pathStartEncoderLeft = getEncoderLeftCnt();   // Record encoder count when path starts
    pathStartEncoderRight = getEncoderRightCnt(); // Record encoder count when path starts

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
    float dt = (currentTime - lastPIDTime) / 1000.0f;
    if (dt <= 0) { dt = 0.001f; }
    lastPIDTime = currentTime;

    // PID calculation
    float error = float(linePos) - GOAL;
    
    // Proportional term
    float P = KP * error;
    
    // Integral term with hard clamp (windup protection)
    integral += error * dt;
    if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
    if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
    float I = KI * integral;
    
    // Derivative term
    float D = KD * ((error - lastError) / dt);
    lastError = error;
    
    // Total PID output
    float delta = P + I + D;
    
    // Ensure motors are forward before applying speed
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);

    // Determine current speed based on encoder counts and whether we've shot yet
    int currentBaseSpeed = BASE_SPEED;
    
    if (!shot) {
        // Going to shoot for the first time
        // Calculate average encoder ticks traveled since path started
        uint16_t leftTraveled = getEncoderLeftCnt() - pathStartEncoderLeft;
        uint16_t rightTraveled = getEncoderRightCnt() - pathStartEncoderRight;
        uint16_t avgTraveled = (leftTraveled + rightTraveled) / 2;
        
        if (avgTraveled >= SLOWDOWN_ENCODER_TICKS) {
            // After threshold encoder ticks, slow down until bumper is hit
            currentBaseSpeed = SLOW_SPEED;
        }
    }
    // After shooting (shot == true), always use normal BASE_SPEED on the way back

    // Apply PID correction to base speed
    int left_motor_speed = constrain(int(currentBaseSpeed + delta), 0, MAX_SPEED);
    int right_motor_speed = constrain(int(currentBaseSpeed - delta), 0, MAX_SPEED);

    setMotorSpeed(LEFT_MOTOR, left_motor_speed);
    setMotorSpeed(RIGHT_MOTOR, right_motor_speed);

    if (newBumperPress) {
        setMotorSpeed(BOTH_MOTORS, 0);
        lastError = 0;
        integral = 0;
        
        if (shot) {
            // Second bumper hit - turn around
            delay(50);
            pendingDoneAfterTurn = true;
            state = State::TURN;
        } else {
            // First bumper hit - shoot
            delay(300);
            state = State::SHOOT;
        }
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
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(300);
    digitalWrite(SHOOTER_PIN, LOW);
    delay(10);
    shot = true;
    
    // Reset PID before next move
    lastError = 0;
    integral = 0;
    lastPIDTime = millis();
    
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
        
        // Reset encoders and PID for next path
        resetLeftEncoderCnt();
        resetRightEncoderCnt();

        // Reset PID time so we don't have a huge dt step on resume
        lastPIDTime = millis();
        lastError = 0;
        integral = 0;

        if (pendingDoneAfterTurn) {
            pendingDoneAfterTurn = false;
            state = State::DONE;
        } else {
            // Record encoder counts for return journey at normal speed
            pathStartEncoderLeft = getEncoderLeftCnt();
            pathStartEncoderRight = getEncoderRightCnt();
            state = State::PATH;
        }
    }
}

void done() {
    setMotorSpeed(BOTH_MOTORS, 0);
    if (isButtonPressed() || (isBumperPressed() )) {
        shot = false;
        state = State::RESTART;
    }
}

boolean isBumperPressed() {
    return (isBumpSwitchPressed(2) && isBumpSwitchPressed(3)); // TODO: adjust switch numbers as needed
}

boolean isButtonPressed() {
    return (digitalRead(LP_LEFT_BTN) == 0); // TODO: Change button
}
