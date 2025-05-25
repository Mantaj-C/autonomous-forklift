#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

#include "CMotor.h"
#include "CServo.h"

class CPathing {
private:

    CMotor _motors;
    std::vector<std::vector<cv::Point2f>> NoNoAreas;
    std::vector<CServo> _servos;

public:

    enum servos {BL = 0, BR, ML, MR};

    CPathing(); 
    ~CPathing();   

     enum orientation {
    UPPER = 90,
    LOWER = 270,
    LEFT = 180,
    RIGHT = 0
};

    void ServoLift(int id);
    void ServoLower(int id);
    void servo_adjust_upper_up(int id);
    void servo_adjust_upper_down(int id);
    void servo_adjust_lower_up(int id);
    void servo_adjust_lower_down(int id);

    void move_distance(float distance);
    void move_to_pickup();
    void backup();
    void move_to_dropoff();

    void leave_start_box();

    void right_90();
    void left_90();

    void right_45();
    void left_45();

    void AutoPathing();
    bool AutoRouting(cv::Mat &frame, cv::Point2f target_position, int target_orientation);
    cv::Point2f Car_Position(cv::Mat frame, float &orientation);
    bool TestRouting(cv::Point2f current_position, float distance, float angleDeg);

    void Turn_right(float angle, int PWM_Freq);
    void Turn_left(float angle, int PWM_Freq);
    void Move_forward(float distance, int PWM_Freq);
    void move_backward(float distance, int PWM_Freq);

    void mmperpixel(cv::Mat frame);
    bool orientate(cv::Mat &frame, float angle);
};
