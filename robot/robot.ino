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
const float KP = 0.05;// POGGIES: 0.05, 0.001 up to 0.0012, 0.01
const float KI = 0.001;
const float KD = 0.01;
const double LINE_POSITION_GOAL = 3500.0;
const unsigned long PID_TELEMETRY_INTERVAL_MS = 50;

const int GOAL = 3500;  // Center position for line sensor

float lastError = 0;
float integral = 0;
unsigned long lastPIDTime = 0;

bool bumperPreviouslyPressed = false; // Tracks last bumper state to detect new hits
bool pendingDoneAfterTurn = false;    // Signals that the next turn should end in DONE

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

    bumperPreviouslyPressed = isBumperPressed();
    pendingDoneAfterTurn = false;

    state = State::PATH;
}

void restart() {
    delay(100); // Debounce/Settle delay

    // Reset everything
    resetLeftEncoderCnt();
    resetRightEncoderCnt();

    // Initialize motors
    enableMotor(BOTH_MOTORS);

    bumperPreviouslyPressed = isBumperPressed();
    pendingDoneAfterTurn = false;

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
        if (shot) {
            setMotorSpeed(BOTH_MOTORS, 0);  // Stop motors
            lastError = 0;
            integral = 0;
            lastPIDTime = millis();
            delay(50);
            pendingDoneAfterTurn = true;
            state = State::TURN;
        } else {
            setMotorSpeed(BOTH_MOTORS, 0);  // Stop motors
            lastError = 0;
            integral = 0;
            lastPIDTime = millis();
            delay(400);
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
    delay(100);
    digitalWrite(SHOOTER_PIN, HIGH);
    delay(300);
    digitalWrite(SHOOTER_PIN, LOW);
    delay(50);
    shot = true;
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

        if (pendingDoneAfterTurn) {
            pendingDoneAfterTurn = false;
            state = State::DONE;
        } else {
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
