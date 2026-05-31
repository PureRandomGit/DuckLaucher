#include "SimpleRSLK.h"

const int BASE_SPEED = 70;              // Motor speed used while running straight
const unsigned long PRINT_INTERVAL = 100; // ms between serial updates

bool running = false;
unsigned long lastReport = 0;
uint16_t prevLeft = 0;
uint16_t prevRight = 0;

void setup() {
	Serial.begin(115200);
	setupRSLK();

	resetLeftEncoderCnt();
	resetRightEncoderCnt();

	setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
	enableMotor(BOTH_MOTORS);
	setMotorSpeed(BOTH_MOTORS, 0);

	pinMode(LP_LEFT_BTN, INPUT_PULLUP);

	Serial.println("Encoder tuning ready");
	Serial.println("Press LEFT button to toggle motors, hold to stop");
	Serial.println("Counts will stream for Serial Plotter in format 'Left:x Right:y'\n");
}

void loop() {
	// Toggle run state when the left button is pressed (active low)
	if (digitalRead(LP_LEFT_BTN) == LOW) {
		running = !running;
		delay(250); // debounce and avoid repeated toggles
		if (!running) {
			setMotorSpeed(BOTH_MOTORS, 0);
			Serial.println("Stopped. Press again to resume");
			return;
		} else {
			resetLeftEncoderCnt();
			resetRightEncoderCnt();
			prevLeft = 0;
			prevRight = 0;
			Serial.println("Running. Counts reset");
		}
	}

	if (running) {
		setMotorSpeed(LEFT_MOTOR, BASE_SPEED);
		setMotorSpeed(RIGHT_MOTOR, BASE_SPEED);
	}

	unsigned long now = millis();
	if (now - lastReport >= PRINT_INTERVAL) {
		lastReport = now;

		uint16_t leftCount = getEncoderLeftCnt();
		uint16_t rightCount = getEncoderRightCnt();

		int leftDelta = leftCount - prevLeft;
		int rightDelta = rightCount - prevRight;
		prevLeft = leftCount;
		prevRight = rightCount;

		// Plotter/monitor friendly output
		Serial.print("Left:");
		Serial.print(leftCount);
		Serial.print(" Right:");
		Serial.print(rightCount);
		Serial.print(" dLeft:");
		Serial.print(leftDelta);
		Serial.print(" dRight:");
		Serial.println(rightDelta);
	}
}
