#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorControl {
public:
    MotorControl();
    ~MotorControl();

    void initializePWM();
    void setMotorSpeeds(int leftSpeed, int rigthSpeed);

private:
    const int leftMotorPin = 13;  // GPIO 13 (PWM0) black
    const int leftMotordir = 6;  // GPIO 6 yellow
    const int rigthMotorPin = 12;  // GPIO 12 (PWM1) grey
    const int rigthMotordir = 5;  // GPIO 5 orange

};

#endif // MOTOR_CONTROLLER_H