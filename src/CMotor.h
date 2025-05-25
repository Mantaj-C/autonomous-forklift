#pragma once

#include <pigpio.h>
#include <opencv2/opencv.hpp>

class CMotor
{
private:

    int _PWM_FREQ;

public:

    CMotor();
    ~CMotor();

    void forward();
    void reverse();

    void left();
    void right();

    void start();
    void stop();

    void setPWM(int PWM);
    int getFrequency();

    void servoLift(int height, int pin);
    void servoLower(int height, int pin);

    void TravelTiming(float distance_cm);
    void TurnTiming(float angle_deg);

    void MotorTest();
    void RampUp(int startPWM, int endPWM, int duration);

};

