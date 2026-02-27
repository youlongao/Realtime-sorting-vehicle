#include <iostream>
#include <csignal>
#include <pigpio.h>
#include "motor_controller.h"
#include "speed_controller.h"

MotorController* g_motor = nullptr;
SpeedController* g_speedCtrl = nullptr;

void signalHandler(int sig) {
    if (g_speedCtrl) g_speedCtrl->stop();
    gpioTerminate();
    exit(0);
}

int main() {
    if (gpioInitialise() < 0) {
        return 1;
    }

    signal(SIGINT, signalHandler);

    MotorController motor(18, 23, 24, 25, 8, 7);

    if (!motor.initialize()) {
        gpioTerminate();
        return 1;
    }

    SpeedController speedCtrl(motor, 17, 27, 22, 10, 600, 0.065, 0.05);

    if (!speedCtrl.initialize()) {
        gpioTerminate();
        return 1;
    }

    speedCtrl.setTargetSpeed(0.3, 0.3);

    speedCtrl.start();
    //Ctrl+C ֹͣ

    while (true) {
        gpioDelay(1000000);
    }
    return 0;
}
