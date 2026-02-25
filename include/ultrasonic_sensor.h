#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "share_types.h"
#include <cstdint>

class Ultrasonic_Sensor
{
public:
	Ultrasonic_Sensor(int trigPin, int echoPin);
	~Ultrasonic_Sensor();

	bool initialize();

	// callback function after registation distance calculation is completed
	void registerCallback(DistanceCallback callback);

	// triggering ultrasonic pulses
	void trigger();

private:
	int trigPin;
	int echoPin;
	DistanceCallback m_callback;
	uint32_t startTick;	// record timestamp

	// pigpio requests a C-style stastic interrupt callback function
	static void echoInterruptWrapper(int gpio, int level, uint32_t tick, void* userdata);

	// actual C++ interrupt handling logic
	void handleEchoInterrupt(int level, uint32_t tick);

};

#endif