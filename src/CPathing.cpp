#include "CPathing.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/dnn.hpp>

#define ML_LO 1725
#define MR_LO 1525
#define BL_LO 2075
#define BR_LO 2100

#define ML_HI 1825
#define MR_HI 1625
#define BL_HI 2175
#define BR_HI 2200

CPathing::CPathing() {
    _servos.push_back(CServo(21,true, BL_HI, BL_LO));
    _servos.push_back(CServo(7,false, BR_HI, BR_LO));
    _servos.push_back(CServo(8,true, ML_HI, ML_LO));
    _servos.push_back(CServo(20,false, MR_HI, MR_LO));
    NoNoAreas.push_back({cv::Point2f(0,0), cv::Point2f(1170,0), cv::Point2f(1170,20), cv::Point2f(0,20)});
    NoNoAreas.push_back({cv::Point2f(0,0), cv::Point2f(1170,0), cv::Point2f(1170,20), cv::Point2f(0,20)});
}

CPathing::~CPathing() { 

}

#define ninety_degrees 13.27
#define MMPERPIXEL 1.95791666667
#define Orientation_error 10
#define distance_error 10
#define MAX_SINGLE_MOVE 100

#define MAX_SINGLE_TURN 10
#define DISTANCE_THRESHOLD 100


#define SERVO_ADJUSTMENT 5

#define FAST_FOWARD_FREQ 240
#define FORWARD_FREQ 240
#define FAST_TURN_FREQ 60
#define SLOW_TURN_FREQ 5

#define FINE_TURN

void CPathing::ServoLift(int id) {
    _motors.servoLift(_servos[id].get_upper(), _servos[id].get_pin());
}

void CPathing::ServoLower(int id) {
    _motors.servoLower(_servos[id].get_lower(), _servos[id].get_pin());
}

void CPathing::move_distance(float distance) {
    _motors.forward();
    _motors.start();
    _motors.TravelTiming(distance);
    _motors.stop();
}

void CPathing::servo_adjust_upper_up(int id) {
    _servos[id].adjust_upper(SERVO_ADJUSTMENT);
    std::cout << "Upper Servo: " << _servos[id].get_upper() << std::endl;
}

void CPathing::servo_adjust_upper_down(int id) {
    _servos[id].adjust_upper(-SERVO_ADJUSTMENT);
    std::cout << "Upper Servo: " << _servos[id].get_upper() << std::endl;
}

void CPathing::servo_adjust_lower_up(int id) {
    _servos[id].adjust_lower(SERVO_ADJUSTMENT);
    std::cout << "Lower Servo: " << _servos[id].get_lower() << std::endl;
}

void CPathing::servo_adjust_lower_down(int id) {
    _servos[id].adjust_lower(-SERVO_ADJUSTMENT);
    std::cout << "Lower Servo: " << _servos[id].get_lower() << std::endl;
}

void CPathing::AutoPathing() {
    
    std::cout << "Auto Pathing" << std::endl;
    std::cout << "Press 's' to start" << std::endl;

    std::string input;
    std::cin >> input;
    
    if (input == "s") {
        std::cout << "Starting..." << std::endl;
        leave_start_box();
        right_90();
        move_distance(31);
        left_90();
        move_distance(100);
    }


}

bool CPathing::AutoRouting(cv::Mat &frame, cv::Point2f target_position, int target_orientation) {
    int checks = 0;
    float delta_move = 0;
    for (const auto& polygon : NoNoAreas) {
        // Convert mm to pixels for drawing
        std::vector<cv::Point> pts_px;
        for (const auto& pt_mm : polygon) {
            pts_px.emplace_back(
                static_cast<int>(pt_mm.x / MMPERPIXEL),
                static_cast<int>(pt_mm.y / MMPERPIXEL)
            );
        }
    
        // Create mask for alpha blending
        cv::Mat overlay = frame.clone();
        const cv::Scalar orange(0, 165, 255); // BGR for orange
        std::vector<std::vector<cv::Point>> fill_poly = { pts_px };
        cv::fillPoly(overlay, fill_poly, orange);
    
        // Alpha blend with 50% opacity
        double alpha = 0.5;
        cv::addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame);
    }
    std::cout << "Auto Routing to Target (" << target_position.x << ", " << target_position.y << ")" << std::endl;
    float current_orientation;
    cv::Point2f current_position = Car_Position(frame, current_orientation);
    if (current_position.x == 0 && current_position.y == 0) {
        return false;
    }
    
    float delta_x = target_position.x - current_position.x;
    float delta_y = target_position.y - current_position.y;
    float distance = sqrt(delta_x*delta_x + delta_y*delta_y);
    
    float angleRad = atan2(delta_x, delta_y);               // Range: [-π, π]
    float angleDeg = angleRad * 180.0 / M_PI;     // Convert to degrees
    angleDeg = angleDeg - 90;
    if (angleDeg < 0) { angleDeg += 360.0; }
    float delta_angle = angleDeg - current_orientation;
    

    std::cout << "Angle of travel line: " << angleDeg << std::endl;
    std::cout << "Current Orientation: " << current_orientation << std::endl;
    std::cout << "Delta Angle: " << delta_angle << std::endl;
    
    cv::circle(frame, current_position / MMPERPIXEL, 10, cv::Scalar(0, 255, 0), -1);
    cv::circle(frame, target_position / MMPERPIXEL, 10, cv::Scalar(0, 0, 255), -1);
    
    if (TestRouting(current_position, distance, angleDeg)) {
        cv::line(frame, current_position / MMPERPIXEL, target_position / MMPERPIXEL, cv::Scalar(255, 0, 0), 2);
        if (distance >= distance_error) {
            float angle_error = 0;
            if (distance >= DISTANCE_THRESHOLD) {
                angle_error = Orientation_error;
            }
            else {
                angle_error = Orientation_error * (4/3);
            }
            if (abs(delta_angle) >= angle_error) {
                if (delta_angle >= 180 || ((delta_angle <= 0) && (delta_angle > -180))) {
                    
                    delta_angle = 360 - delta_angle;
                    if (abs(delta_angle) > MAX_SINGLE_TURN) {
                        delta_angle = MAX_SINGLE_TURN;
                    }

                    if(angle_error == Orientation_error) {
                        Turn_right(abs(delta_angle), FAST_TURN_FREQ);
                    } else if (angle_error == Orientation_error * (4/3)) {
                        Turn_right(0.1, SLOW_TURN_FREQ);
                    }

                    std::cout << "Turning right: " << delta_angle <<std::endl;
                    current_orientation -= delta_angle;
                }
                else {
                    if (abs(delta_angle) > MAX_SINGLE_TURN) {
                       delta_angle = MAX_SINGLE_TURN;
                    }
                    
                      if(angle_error == Orientation_error) {
                        Turn_left(abs(delta_angle), FAST_TURN_FREQ);
                    } else if (angle_error == Orientation_error *(4/3)) {
                        Turn_left(0.1, SLOW_TURN_FREQ);
                    }

                    std::cout << "Turning left: " << delta_angle <<std::endl;
                    current_orientation += delta_angle;
                }
            }
            else {

                if (distance >= MAX_SINGLE_MOVE) {
                    Move_forward(MAX_SINGLE_MOVE, FORWARD_FREQ);
                }
                else {
                    Move_forward(distance, FORWARD_FREQ);
                }
            }
        }
        else {
            checks += 2;
        }
    }
    else {
        float delta_x = target_position.x - current_position.x;
        float delta_y = target_position.y - current_position.y;
        if (delta_x >= distance_error) {
            if (delta_x >= 0) {
                if (current_orientation >= 180) {
                    Turn_left(360 - current_orientation, FAST_TURN_FREQ);
                }
                else {
                    Turn_right(current_orientation, FAST_TURN_FREQ);
                }
                current_orientation = 0;
                Move_forward(delta_x, FORWARD_FREQ);
            }
            else {
                if (current_orientation >= 180) {
                    Turn_right(current_orientation - 180, FAST_TURN_FREQ);
                }
                else {
                    Turn_left(180 - current_orientation, FAST_TURN_FREQ);
                }
                current_orientation = 180;
                Move_forward(-1 * delta_x, FORWARD_FREQ);
            }
        }
        else {
            checks++;
        }
        if (delta_y >= distance_error) {
            if (delta_y >= 0) {
                if (current_orientation == 0) {
                    Turn_right(90, FAST_TURN_FREQ);
                }
                else {
                    Turn_left(90, FAST_TURN_FREQ);
                }
                Move_forward(delta_y, FORWARD_FREQ);
            }
            else {
                if (current_orientation == 0) {
                    Turn_left(90, FAST_TURN_FREQ);
                }
                else {
                    Turn_right(90, FAST_TURN_FREQ);
                }
                Move_forward(-1 * delta_y, FORWARD_FREQ);
            }
        }
        else {
            checks++;
        }
    }
    if (checks >= 2) {
        return true;
    }
    else {
        return false;
    }
}

cv::Point2f CPathing::Car_Position(cv::Mat frame, float &orientation) {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams);
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    if (markerCorners.empty()) {
        return cv::Point2f(0, 0);
    }
    float dx = markerCorners[0][1].x - markerCorners[0][0].x;
    float dy = markerCorners[0][1].y - markerCorners[0][0].y;

    float angleRad = atan2(dx, dy);               // Range: [-π, π]
    float angleDeg = angleRad * 180.0 / M_PI;     // Convert to degrees
    
    if (angleDeg < 0)
    angleDeg += 360.0;      
    std::cout << "Angle: " << angleDeg << std::endl;
    orientation = angleDeg;
    
    cv::Point2f middle = cv::Point2f((markerCorners[0][0].x + markerCorners[0][2].x)/2, (markerCorners[0][0].y + markerCorners[0][2].y)/2);
    cv::line (frame, middle, cv::Point2f((markerCorners[0][0].x + markerCorners[0][1].x)/2, (markerCorners[0][0].y + markerCorners[0][1].y)/2), cv::Scalar(0, 255, 0), 2);
    return cv::Point2f((markerCorners[0][0].x + markerCorners[0][2].x)/2 * MMPERPIXEL, (markerCorners[0][0].y + markerCorners[0][2].y)/2 * MMPERPIXEL);
}

void CPathing::Turn_right(float angle, int PWM_Freq) {
    std::cout << "Turning right: " << angle << std::endl;
    _motors.setPWM(PWM_Freq);
    _motors.right();
    _motors.start();
    _motors.TravelTiming((ninety_degrees/90)*angle);
    _motors.stop();
    _motors.setPWM(PWM_Freq);
    std::cout << "Right Turn Complete" << std::endl;
}

void CPathing::Turn_left(float angle, int PWM_Freq) {
    std::cout << "Turning left: " << angle << std::endl;
    _motors.left();
    _motors.start();
    _motors.TravelTiming((ninety_degrees/90)*angle);
    _motors.stop();
    std::cout << "Left Turn Complete" << std::endl;
}

void CPathing::Move_forward(float distance, int PWM_Freq) {
    std::cout << "Moving forward" << std::endl;
    _motors.forward();
    _motors.start();
    _motors.TravelTiming(distance/10);
    std::cout << "Done Timing" << std::endl;
    _motors.stop();
    std::cout << "Forward Move Complete" << std::endl;
}

void CPathing::move_backward(float distance, int PWM_Freq) {
    std::cout << "Moving backward" << std::endl;
    _motors.reverse();
    _motors.start();
    _motors.TravelTiming(distance/10);
    _motors.stop();
    std::cout << "Backward Move Complete" << std::endl;
}

void CPathing::mmperpixel(cv::Mat frame) {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<int> markerIds;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams);
    cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    float area = (((markerCorners[0][0].x * markerCorners[0][1].y) - (markerCorners[0][1].x * markerCorners[0][0].y)) + 
    ((markerCorners[0][1].x * markerCorners[0][2].y) - (markerCorners[0][2].x * markerCorners[0][1].y)) + 
    ((markerCorners[0][2].x * markerCorners[0][3].y) - (markerCorners[0][3].x * markerCorners[0][2].y)) +
    ((markerCorners[0][3].x * markerCorners[0][0].y) - (markerCorners[0][0].x * markerCorners[0][3].y)));
    std::cout << "MM per Pixel: " << area / 150*150 << std::endl;
}

bool CPathing::TestRouting(cv::Point2f current_position, float distance, float angleDeg) {
    // Robot dimensions in mm
    const float robotLength = 190.0f;
    const float robotWidth = 180.0f;

    // Compute target position
    float angleRad = angleDeg * CV_PI / 180.0f;
    cv::Point2f offset(std::cos(angleRad) * distance, std::sin(angleRad) * distance);
    cv::Point2f target_position = current_position + offset;

    // Create initial and final oriented rectangles (as RotatedRect)
    cv::RotatedRect rectStart(current_position, cv::Size2f(robotWidth, robotLength), angleDeg);
    cv::RotatedRect rectEnd(target_position, cv::Size2f(robotWidth, robotLength), angleDeg);

    // Get corners of both rectangles
    std::vector<cv::Point2f> startCorners(4), endCorners(4);
    rectStart.points(startCorners.data());
    rectEnd.points(endCorners.data());

    // Merge into swept polygon (convex hull of all 8 corners)
    std::vector<cv::Point2f> sweptArea = startCorners;
    sweptArea.insert(sweptArea.end(), endCorners.begin(), endCorners.end());

    std::vector<cv::Point2f> hull;
    cv::convexHull(sweptArea, hull);

    // Check intersection with each NoNoArea polygon
    for (const auto& noNoPolygon : NoNoAreas) {
        std::vector<cv::Point2f> intersection;
        int result = cv::intersectConvexConvex(hull, noNoPolygon, intersection);
        if (result == cv::INTERSECT_FULL || result == cv::INTERSECT_PARTIAL) {
            return false; // Collision detected
        }
    }

    return true; // Safe path
}

// Orientate the car to the target angle
// Returns true if the car is orientated to the target angle
// Returns false if the car is not orientated to the target angle
bool CPathing::orientate(cv::Mat &frame, float angle) {
    float current_orientation;
    cv::Point2f current_position = Car_Position(frame, current_orientation); 
    delta_angle = angle - current_orientation;
    if (deltaangle >= orentation_error) {
        if (delta_angle >= 180 || ((delta_angle <= 0) && (delta_angle > -180))) {
            Turn_right(abs(delta_angle), FAST_TURN_FREQ);
        }
        else{
            Turn_left(abs(delta_angle), FAST_TURN_FREQ);
        }
    }
    else {
        return true;
    }
}
