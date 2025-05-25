#include "CMotor.h"
#include <iostream>
#include <chrono>
#include <thread>

//PWM Settings:
#define PWM_FREQ 120
// 10ms Period
#define PWM_HIGH 128 // Range 0-255, 128 is 50% duty cycle
#define PWM_LOW 0

//Left Motor Pins:
#define ENABLE_LEFT_PIN 11
#define MOTOR_LEFT_PIN 9
#define DIRECTION_LEFT_PIN 10

//#define EN_LEFT_PIN 18

//Right Motor Pins:
#define ENABLE_RIGHT_PIN 22
#define MOTOR_RIGHT_PIN 27
#define DIRECTION_RIGHT_PIN 17

//Lift Servo Stuff:

#define SERVO_PIN 18

#define SERVO_RAISE 1325
#define SERVO_LOWER 1750



enum  motorDirection {CCW = 0, CW };
enum  motorState {ENABLED = 0, DISABLED };

CMotor::CMotor() {

    //Initialise GPIO:

    _PWM_FREQ = PWM_FREQ;

    //Set the gpio pins:
    gpioSetMode(ENABLE_LEFT_PIN, PI_OUTPUT);
    gpioSetMode(ENABLE_RIGHT_PIN, PI_OUTPUT);

    gpioSetMode(DIRECTION_LEFT_PIN, PI_OUTPUT);
    gpioSetMode(DIRECTION_RIGHT_PIN, PI_OUTPUT);

    gpioSetMode(MOTOR_LEFT_PIN, PI_OUTPUT);
    gpioSetMode(MOTOR_RIGHT_PIN, PI_OUTPUT);

    gpioSetMode(SERVO_PIN, PI_OUTPUT);

    //Diable Motors:
    gpioWrite(ENABLE_LEFT_PIN, ENABLED);
    gpioWrite(ENABLE_RIGHT_PIN, ENABLED);
    
    //Set PWM Frequency:
    gpioSetPWMfrequency(MOTOR_LEFT_PIN, PWM_FREQ); 
    gpioSetPWMfrequency(MOTOR_RIGHT_PIN, PWM_FREQ);

    gpioPWM(MOTOR_LEFT_PIN, PWM_LOW);
    gpioPWM(MOTOR_RIGHT_PIN, PWM_LOW);

}

CMotor::~CMotor() {
    //Shut off Motors:
    stop();
    std::cout << "Stopping Motors..." << std::endl;
}


void CMotor::servoLift(int height,int pin) {
    //Set Servo to 180 degrees:
    gpioServo(pin, height);
}

void CMotor::servoLower(int height,int pin) {
    //Set Servo to 0 degrees:
    gpioServo(pin, height);
}   

void CMotor::forward() {

    //Set Motor Direction:
    gpioWrite(DIRECTION_LEFT_PIN, CW);
    gpioWrite(DIRECTION_RIGHT_PIN, CCW);
}

void CMotor::reverse() {

    //Set Motor Direction:
    gpioWrite(DIRECTION_LEFT_PIN, CCW);
    gpioWrite(DIRECTION_RIGHT_PIN, CW);
}

void CMotor::left() {

    //Set Motor Direction:
    gpioWrite(DIRECTION_LEFT_PIN, CCW);
    gpioWrite(DIRECTION_RIGHT_PIN, CCW);
}

void CMotor::right() {

    //Set Motor Direction:
    gpioWrite(DIRECTION_LEFT_PIN, CW);
    gpioWrite(DIRECTION_RIGHT_PIN, CW);
}

void CMotor::start() {

    //Enable Motors:
    //gpioWrite(ENABLE_LEFT_PIN, ENABLED);
    //gpioWrite(ENABLE_RIGHT_PIN, ENABLED);

    gpioPWM(MOTOR_LEFT_PIN, PWM_HIGH);
    gpioPWM(MOTOR_RIGHT_PIN, PWM_HIGH);
}

void CMotor::stop() {

    //Disable Motors:
    //gpioWrite(ENABLE_LEFT_PIN, DISABLED);
    //gpioWrite(ENABLE_RIGHT_PIN, DISABLED);

    gpioPWM(MOTOR_LEFT_PIN, PWM_LOW);
    gpioPWM(MOTOR_RIGHT_PIN, PWM_LOW);
}

void CMotor::setPWM(int PWM) {

    _PWM_FREQ = PWM;
    gpioSetPWMfrequency(MOTOR_LEFT_PIN, _PWM_FREQ);
    gpioSetPWMfrequency(MOTOR_RIGHT_PIN, _PWM_FREQ);

}

int CMotor::getFrequency() {return _PWM_FREQ; }

#define radius_cm 3.46
#define PI_2 6.283185307179586 // 2 x Pi

#define steps_per_cm (200)/(PI_2 * radius_cm)  

void CMotor::TravelTiming(float distance_cm) {
    int time_ms = distance_cm * steps_per_cm * (1000/_PWM_FREQ); // Distance x Steps/Distance x Time/Step
    std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    //std::cout << "Time Delay: " << time_ms << std::endl;
}

#define steps_per_deg (200/360) // 200 Steps per 360 Degrees

void CMotor::TurnTiming(float angle_deg) {
    int time_ms = angle_deg * steps_per_deg* (1000/_PWM_FREQ); // Angle x Steps/Angle x Time/Step
    std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
}

void CMotor::MotorTest() {

    std::cout << "Motor Test:" << std::endl;
    bool exit = false;

    do {

        std::cout << "Press 'w' to forward" << std::endl;
        std::cout << "Press 's' to reverse" << std::endl;
        std::cout << "Press 'a' to left" << std::endl;
        std::cout << "Press 'd' to right" << std::endl;
        std::cout << "Press 'e' to start" << std::endl;
        std::cout << "Press 'r' to stop" << std::endl;
        std::cout << "Press '+' to increase PWM Frequency" << std::endl;
        std::cout << "Press '-' to decrease PWM Frequency" << std::endl;
        std::cout << "Press 'q' to quit" << std::endl;
        std::string input;
        std::cin >> input;

        if (input == "w") {
            forward();
            setPWM(0);
            start();
            RampUp(0, 200, 10);
            TravelTiming(30.48); // Forward 1cm
            stop();
        } else if (input == "s") {
            reverse();
            start();
            TravelTiming(1); // Reverse 1cm
            stop();
        } else if (input == "a") {
            left();
            start();
            TurnTiming(4); // Turn 1deg left
            stop();
        } else if (input == "d") {
            right();
            start();
            TurnTiming(45); // Turn 1deg right
            stop();
        } else if (input == "e") {
            start();
        } else if (input == "r") {
            stop();
        } else if (input == "+") {
            setPWM(_PWM_FREQ + 10);
            std::cout << "PWM Frequency: " << getFrequency() << std::endl;
        } else if (input == "-") {  
            setPWM(_PWM_FREQ - 10);
            std::cout << "PWM Frequency: " << getFrequency() << std::endl;
        } else if (input == "q") {
            exit = true;
            std::cout << "Exiting..." << std::endl;
        }

    } while (exit == false);
    
}


void CMotor::RampUp(int startPWM, int endPWM, int duration_ms) {
    int time_ms = duration_ms;
    for (int i = startPWM; i <= endPWM; i++) {
        setPWM(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
    }
}