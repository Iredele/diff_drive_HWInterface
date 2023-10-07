#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <wiringPi.h>
#include <algorithm>

class MotorControl {
public:
    MotorControl() {
        leftMotorPin = 13;     // GPIO 13 (PWM0) black
        leftMotordir = 6;     // GPIO 6 yellow
        rigthMotorPin = 12;   // GPIO 12 (PWM1) grey
        rigthMotordir = 5;    // GPIO 5 orange
    }

    ~MotorControl() {}

    void initializePWM() {
        wiringPiSetupGpio();
        pinMode(leftMotorPin, PWM_OUTPUT);
        pinMode(rigthMotorPin, PWM_OUTPUT);
        pinMode(leftMotordir, OUTPUT);
        pinMode(rigthMotordir, OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(100);
        pwmSetClock(384);
    }

    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        bool dir1, dir2;

        if (leftSpeed >= 0) {
            dir1 = LOW;
        } else {
            dir1 = HIGH;
        }

        if (rightSpeed >= 0) {
            dir2 = HIGH;
        } else {
            dir2 = LOW;
        }

        digitalWrite(leftMotordir, dir1);
        digitalWrite(rigthMotordir, dir2);

        leftSpeed = std::max(std::min(leftSpeed, 1), 0);
        rightSpeed = std::max(std::min(rightSpeed, 1), 0);

        int leftPwm = leftSpeed * 100;
        int rightPwm = rightSpeed * 100;

        pwmWrite(leftMotorPin, leftPwm);
        pwmWrite(rigthMotorPin, rightPwm);
    }

private:
    int leftMotorPin;
    int leftMotordir;
    int rigthMotorPin;
    int rigthMotordir;
};

#endif // MOTOR_CONTROLLER_H
