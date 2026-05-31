#include "SimpleRSLK.h"

const int BASE_SPEED = 100;
const float KP = 0.5; // Proportional gain for encoder correction

void setup() {
    Serial.begin(115200);
    setupRSLK();
    
    resetLeftEncoderCnt();
    resetRightEncoderCnt();
    
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
    enableMotor(BOTH_MOTORS);
    
    delay(1000);
    Serial.println("Going straight...");
}

void loop() {
    // Read encoder counts
    uint16_t leftCount = getEncoderLeftCnt();
    uint16_t rightCount = getEncoderRightCnt();
    
    // Calculate difference (positive means left wheel is ahead)
    int error = leftCount - rightCount;
    
    // Correct by adjusting motor speeds
    int correction = KP * error;
    
    int leftSpeed = constrain(BASE_SPEED - correction, 0, 100);
    int rightSpeed = constrain(BASE_SPEED + correction, 0, 100);
    
    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);
    
    // Serial Plotter format: label:value label:value
    Serial.print("Left:");
    Serial.print(leftCount);
    Serial.print(" Right:");
    Serial.print(rightCount);
    Serial.print(" Error:");
    Serial.println(error);
    
    delay(10); // Small delay for stability
}
