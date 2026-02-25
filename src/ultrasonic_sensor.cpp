#include "ultrasonic_sensor.h"
#include <pigpio.h>
#include <iostream>

Ultrasonic_Sensor::Ultrasonic_Sensor(int trigPin, int echoPin)
	: trigPin(trigPin), echoPin(echoPin), startTick(0){ }

Ultrasonic_Sensor::~Ultrasonic_Sensor() 
{
	// cancel interrupt binding to prevent callback leaks
	gpioSetAlertFuncEx(echoPin, nullptr, nullptr);
}

bool Ultrasonic_Sensor::initialize() 
{
	std::cout << "initialize the ultrasonic sensor pins..." << std::endl;
	gpioSetMode(trigPin, PI_OUTPUT);  // send ultrasonic trigger signal
	gpioSetMode(echoPin, PI_INPUT);	  // receive echo
	gpioSetMode(trigPin, 0);	// ensure the tirgger pin is initially at a low level

	// register hardware edge-trigger interrupt
	// automatically invoked when pin level chages
	gpioSetAlertFuncEx(echoPin, echoInterruptWrapper, this);

	return true;
}

void Ultrasonic_Sensor::registerCallback(DistanceCallback callback)
{
	m_callback = std::move(callback);
}

void Ultrasonic_Sensor::trigger()
{
	// send a 10 microseconds high pulse 
	gpioTrigger(trigPin, 10, 1);
}

void Ultrasonic_Sensor::echoInterruptWrapper(int gpio, int level, uint32_t tick, void* userdata)
{
	Ultrasonic_Sensor* sensor = static_cast<Ultrasonic_Sensor*>(userdata);
	if (sensor) {
		sensor -> handleEchoInterrupt(level, tick);
	}
}

void Ultrasonic_Sensor::handleEchoInterrupt(int level, uint32_t tick)
{
	if (level == 1) {
		startTick = tick;	// record rising edge time
	}
	else if (level == 0) {
		uint32_t diff = tick - startTick;	// calculate the time difference on the falling edge
		float distance = (diff * 0.0343f) / 2.0f;	// calculate distance - cm

		if (m_callback) {
			m_callback(distance);
		}
	}
}
