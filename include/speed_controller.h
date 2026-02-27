#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H


#include "motor_controller.h"
#include <pigpio.h>
#include <functional>
#include <atomic>

class SpeedController {
public:
    SpeedController(MotorController& motor,
                    int left_enc_a, int left_enc_b,
                    int right_enc_a, int right_enc_b,
                    float pulses_per_rev,
                    float wheel_diameter,
                    float control_interval = 0.05);

    ~SpeedController();

    bool initialize();

    void setTargetSpeed(float left_mps, float right_mps);

    void start();

    void stop();

private:
    MotorController& motor_;

    int left_enc_a_, left_enc_b_;
    int right_enc_a_, right_enc_b_;

    float pulses_per_rev_;
    float wheel_diameter_;
    float distance_per_pulse_;

    float control_interval_;
    float kp_ = 2.0;
    float target_speed_left_ = 0.0;
    float target_speed_right_ = 0.0;

    std::atomic<long> left_pulse_count_{0};
    std::atomic<long> right_pulse_count_{0};

    long last_left_pulse_ = 0;
    long last_right_pulse_ = 0;

    std::atomic<bool> running_{false};
    int timer_id_ = -1;

    static void leftEncoderISR(int gpio, int level, uint32_t tick, void* userdata);
    static void rightEncoderISR(int gpio, int level, uint32_t tick, void* userdata);

    static void timerCallbackStatic(int id, void* userdata);

    void doControl();
};

#endif // SPEED_CONTROLLER_H
