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
/*
    void setMotorSpeeds(int leftSpeed, int rightSpeed) {
        bool dir1, dir2;

        if (leftSpeed < 0) {
            //dir1 = LOW;
            dir1 = HIGH;
        } else if (leftSpeed > 255){
            //dir1 = HIGH;
            dir1 = LOW;
        }

        if (rightSpeed < 0) {
            //dir2 = HIGH;
            dir2 = HIGH;
        } else if (rightSpeed > 255){
            //dir2 = LOW;
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
    */


void setMotorSpeeds(double leftSpeed, double rightSpeed) {
    // Determine direction based on the sign
    int leftDir = (leftSpeed >= 0) ? LOW : HIGH;
    int rightDir = (rightSpeed >= 0) ? LOW : HIGH;

    // Convert speed values to the absolute range [0, 1]
    leftSpeed = std::abs(leftSpeed);
    rightSpeed = std::abs(rightSpeed);

    // Ensure speed values are within the valid range [0, 1]
    leftSpeed = std::max(std::min(leftSpeed, 1.0), 0.0);
    rightSpeed = std::max(std::min(rightSpeed, 1.0), 0.0);

    // Map speed to PWM range [0, 1000]
    int leftPwm = static_cast<int>(leftSpeed * 1000);
    int rightPwm = static_cast<int>(rightSpeed * 1000);

    // Set direction
    digitalWrite(leftMotordir, leftDir);
    digitalWrite(rigthMotordir, rightDir);

    // Set PWM values
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
