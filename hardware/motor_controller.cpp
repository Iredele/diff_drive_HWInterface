#include "motor_controller.h"
#include <wiringPi.h>

MotorControl::MotorControl() {
    // Constructor (if needed)
}

MotorControl::~MotorControl() {
    // Destructor (if needed)
}

void MotorControl::initializePWM() {
    wiringPiSetupGpio();
    pinMode(leftMotorPin, PWM_OUTPUT);
    pinMode(rightMotorPin, PWM_OUTPUT);
    pinMode(leftMotordir, OUTPUT);
    pinMode(rightMotordir, OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(100);
    pwmSetClock(384);
}

void MotorControl::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if(leftSpeed >= 0){
        bool dir1 = HIGH;
    } else {
        bool dir2 = LOW;

    }

    if(rightSpeed >= 0){
        bool dir2 = HIGH;
    } else {
        bool dir2 = LOW;
    }

    digitalWrite(leftMotordir, dir1);
    digitalWrite(rightMotordir, dir2);

    leftSpeed = std::max(std::min(leftSpeed, 1.0), 0.0);
    rightSpeed = std::max(std::min(rightSpeed, 1.0), 0.0);

    int leftPwm = leftSpeed * 100;
    int rightPwm = rightSpeed * 100;

    pwmWrite(leftMotorPin, leftPwm);
    pwmWrite(rightMotorPin, rightPwm);

}
