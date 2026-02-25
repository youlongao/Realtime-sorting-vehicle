#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorController
{
public:
	// ENA, IN1, IN2 control left wheel; IN3 IN4 ENB control right wheel
	MotorController(int ena, int in1, int in2, int in3, int in4, int enb);
	~MotorController();

	bool initialize();

	// set the motor speed, parameter range is -255 to 255 (negative number represent reversing)
	void setSpeed(int leftSpeed, int rightSpeed);

	// enmergency stop
	void stop();

private:
	int enaPin, in1Pin, in2Pin, in3Pin, in4Pin, enbPin;
};


#endif