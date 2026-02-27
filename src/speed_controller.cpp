#include "speed_controller.h"
#include <iostream>
#include <cmath>

SpeedController::SpeedController(MotorController& motor,
                                 int left_enc_a, int left_enc_b,
                                 int right_enc_a, int right_enc_b,
                                 float pulses_per_rev,
                                 float wheel_diameter,
                                 float control_interval)
    : motor_(motor),
      left_enc_a_(left_enc_a), left_enc_b_(left_enc_b),
      right_enc_a_(right_enc_a), right_enc_b_(right_enc_b),
      pulses_per_rev_(pulses_per_rev),
      wheel_diameter_(wheel_diameter),
      control_interval_(control_interval) {
    distance_per_pulse_ = (M_PI * wheel_diameter_) / pulses_per_rev_;
}

SpeedController::~SpeedController() {
    stop();
}

bool SpeedController::initialize() {

    gpioSetMode(left_enc_a_, PI_INPUT);
    gpioSetMode(left_enc_b_, PI_INPUT);
    gpioSetMode(right_enc_a_, PI_INPUT);
    gpioSetMode(right_enc_b_, PI_INPUT);
    gpioSetPullUpDown(left_enc_a_, PI_PUD_UP);
    gpioSetPullUpDown(left_enc_b_, PI_PUD_UP);
    gpioSetPullUpDown(right_enc_a_, PI_PUD_UP);
    gpioSetPullUpDown(right_enc_b_, PI_PUD_UP);

    if (gpioSetISRFunc(left_enc_a_, RISING_EDGE, 0, leftEncoderISR, this) < 0) {
        return false;
    }
    if (gpioSetISRFunc(right_enc_a_, RISING_EDGE, 0, rightEncoderISR, this) < 0) {
        return false;
    }

    return true;
}

void SpeedController::setTargetSpeed(float left_mps, float right_mps) {
    target_speed_left_ = left_mps;
    target_speed_right_ = right_mps;
}

void SpeedController::start() {
    if (running_) return;
    running_ = true;

    left_pulse_count_ = 0;
    right_pulse_count_ = 0;
    last_left_pulse_ = 0;
    last_right_pulse_ = 0;

    timer_id_ = gpioTimerFunc(0, control_interval_ * 1000, timerCallbackStatic, this);
    if (timer_id_ < 0) {
        running_ = false;
    }
}

void SpeedController::stop() {
    running_ = false;
    if (timer_id_ >= 0) {
        gpioTimerFunc(timer_id_, 0, nullptr, nullptr);
        timer_id_ = -1;
    }
    motor_.stop();
}

void SpeedController::leftEncoderISR(int gpio, int level, uint32_t tick, void* userdata) {
    auto* self = static_cast<SpeedController*>(userdata);
    self->left_pulse_count_++;
}

void SpeedController::rightEncoderISR(int gpio, int level, uint32_t tick, void* userdata) {
    auto* self = static_cast<SpeedController*>(userdata);
    self->right_pulse_count_++;
}

void SpeedController::timerCallbackStatic(int id, void* userdata) {
    auto* self = static_cast<SpeedController*>(userdata);
    if (self->running_) {
        self->doControl();
    }
}

void SpeedController::doControl() {
    long left_now = left_pulse_count_.load();
    long right_now = right_pulse_count_.load();
    long left_delta = left_now - last_left_pulse_;
    long right_delta = right_now - last_right_pulse_;
    last_left_pulse_ = left_now;
    last_right_pulse_ = right_now;

    float left_speed = (left_delta * distance_per_pulse_) / control_interval_;
    float right_speed = (right_delta * distance_per_pulse_) / control_interval_;

    float left_error = target_speed_left_ - left_speed;
    float right_error = target_speed_right_ - right_speed;
    float left_output = kp_ * left_error;
    float right_output = kp_ * right_error;

    int left_pwm = static_cast<int>(left_output);
    int right_pwm = static_cast<int>(right_output);

    motor_.setSpeed(left_pwm, right_pwm);
}
