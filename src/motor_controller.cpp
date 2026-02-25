#include "motor_controller.h"
#include <pigpio.h>
#include <iostream>
#include <algorithm>

MotorController::MotorController(int ena, int in1, int in2, int in3, int in4, int enb)
	: enaPin(ena), in1Pin(in1), in2Pin(in2), in3Pin(in3), in4Pin(in4), enbPin(enb){ }

MotorController::~MotorController()
{
	stop();
}

bool MotorController::initialize()
{
	std::cout << "initial motor drive pins..." << std::endl;
	int pins[] = { enaPin, in1Pin, in2Pin, in3Pin, in4Pin, enbPin };
	for (int pin : pins) {
		gpioSetMode(pin, PI_OUTPUT);
		gpioWrite(pin, 0);
	}
	return true;
}

void MotorController::setSpeed(int leftSpeed, int rightSpeed)
{
	// limit PWM range from -255 to 255
	leftSpeed = std::clamp(leftSpeed, -255, 255);
	rightSpeed = std::clamp(rightSpeed, -255, 255);

	// control left wheel's direction
	if (leftSpeed >= 0) {
		gpioWrite(in1Pin, 1);
		gpioWrite(in2Pin, 0);
	}
	else {
		gpioWrite(in1Pin, 0);
		gpioWrite(in2Pin, 1);
		leftSpeed = -leftSpeed;
	}
	gpioPWM(enaPin, leftSpeed);

	// control right wheel's direction
	if (rightSpeed >= 0) {
		gpioWrite(in3Pin, 1);
		gpioWrite(in4Pin, 0);
	}
	else {
		gpioWrite(in3Pin, 0);
		gpioWrite(in4Pin, 1);
		rightSpeed = -rightSpeed;
	}
	gpioPWM(enbPin, rightSpeed);
}

void MotorController::stop()
{
	gpioWrite(in1Pin, 0);
	gpioWrite(in2Pin, 0);
	gpioPWM(enaPin, 0);

	gpioWrite(in3Pin, 0);
	gpioWrite(in4Pin, 0);
	gpioPWM(enbPin, 0);
}