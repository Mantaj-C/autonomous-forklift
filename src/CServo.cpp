#include "CServo.h"


#define ML_LO 1730
#define MR_LO 1525
#define BL_LO 2065
#define BR_LO 2110

#define ML_HI 1600
#define MR_HI 1600
#define BL_HI 1880
#define BR_HI 2205

CServo::CServo(int gpio_pin, bool Mirror, int upper, int lower) {
   
    servo_upper = upper;
    servo_lower = lower;
  
    pin = gpio_pin;
    Mirrored = Mirror;
}

CServo::~CServo() {

}

void CServo::adjust_upper(int move) {
    if (Mirrored) {
        servo_upper -= move;
    }
    else {
        servo_upper += move;
    }
}

void CServo::adjust_lower(int move) {
    if (Mirrored) {
        servo_lower -= move;
    }
    else {
        servo_lower += move;
    }
}

int CServo::get_upper() {
    return servo_upper;
}

int CServo::get_lower() {
    return servo_lower;
}

int CServo::get_pin() {
    return pin;
}
