#include "line_tracker.h"
#include "ultrasonic_sensor.h"
#include "motor_controller.h"
#include "obstacle_avoidance.h"
#include <pigpio.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <signal.h>

// global control flags and synchronization primitives
std::atomic<bool> running(true);
std::mutex mainMutex;	// locks, used to protect shared resources
std::condition_variable cv;	// thread sleep and wake-up mechanism

// a motor control lock prevents the obstacle avoidance thread and line-following thread
// from operating the motor simulaneously, thus avoiding conflicts
std::mutex motorMutex;

// catch the "ctrl+c" signal to achieve exit
void signalHandler(int signum)
{
	std::cout << "\n[system] receive exit signal (" << signum << "), system is shutting down safely" << std::endl;
	running = false;
	cv.notify_all();
}

int main()
{
	std::cout << "---- Start the real-time sorting robot system ----" << std::endl;

	// regist signal process, ecsure the motor doesn't keep running 
	signal(SIGINT, signalHandler);

	//	initialize the gpio library
	if (gpioInitialise() < 0)
	{
		std::cerr << "pigpio initialization failed! Please ensure that you run this program using sudo " << std::endl;

		return -1;
	}

	// Stack allocation instantiation of all core components
	LineTracker tracker(17, 27, 22);
	Ultrasonic_Sensor ultrasonic(23, 24);
	MotorController motor(5, 6, 13, 19, 26, 12); // ENA ,IN1, IN2, IN3, IN4. ENB
	ObstacleAvoidance obstacleSystem(15.0f);	// set safe distance -> 15cm

	// initialize pins
	tracker.initialize();
	ultrasonic.initialize();
	motor.initialize();

	// register alert callback of obstacle avoidance system 
	obstacleSystem.registerAlertCallback([&motor](bool isBlocked) {
		std::lock_guard<std::mutex> lock(motorMutex);
		if (isBlocked) {
			// encounter obstacle, emeragency stop
			motor.stop();
		}
		else {
			// clear obstacle. waiting for next tracking event to drive the motor
			std::cout << "[System] Allow contiuned tracking..." << std::endl;
		}
	});

	// register distance measurement event callback
	// data flow: ultrasonic layer -> distance measurement callback -> obstacle avodance system
	ultrasonic.registerCallback([&obstacleSystem](float distance) {
		// send currently distance to obstacle avoidance system
		obstacleSystem.processDistanceUpdate(distance);
	});

	// register tracking event callback
	tracker.registerCallback([&motor, & obstacleSystem](int direction) {
		std::lock_guard<std::mutex> lock(motorMutex);

		// if the obstacle avoidance system determines that the path ahead is blocked
		// ignore the tracking command
		if (obstacleSystem.isObstacleDetected()) {
			return;
		}

		// define base speed
		const int BASE_SPEED = 150;
		const int TURN_SPEED = 80;

		switch (direction) {
		case 0:		// center, go straight at full speed
			motor.setSpeed(BASE_SPEED, BASE_SPEED);
			break;
		case -1:	// left, correct to the right
			motor.setSpeed(BASE_SPEED, TURN_SPEED);
			break;
		case 1:		// right, correct to the left
			motor.setSpeed(TURN_SPEED, BASE_SPEED);
			break;
		case 2:		// line lose, stop to find line
			motor.stop();
			break;
		}
	});
	
	// Start a lightweigt timer thread that is only responsible for emitting ultrasonic pulses
	std::thread triggerThread([&ultrasonic]() {
		while (running) {
			ultrasonic.trigger();
			std::this_thread::sleep_for(std::chrono::milliseconds(50));	// distance measurement for 50ms
		}
	});

	std::cout << "[System] all hardware events are bound; enter pure event-driven sleep mode..." << std::endl;

	// main thread
	std::unique_lock<std::mutex> lock(mainMutex);
	cv.wait(lock, []{ return !running.load(); });

	// exit safely and clear resources
	std::cout << "[System] Stopping the trigger thread..." << std::endl;
	triggerThread.join();

	std::cout << "[System] Turning off the motor..." << std::endl;
	motor.stop();

	std::cout << "[System] Releasing GPIO resources..." << std::endl;
	gpioTerminate();

	std::cout << "---- System exit safely ----" << std::endl;

	return 0;
}

