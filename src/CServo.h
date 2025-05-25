#pragma once

#include <iostream>
#include <thread>
#include <chrono>

class CServo {
    private:
        int servo_upper;
        int servo_lower;
        int pin;
        bool Mirrored;
        
    public:
        CServo(int gpio_pin, bool Mirror, int upper, int lower);
        ~CServo();

        void adjust_upper(int move);
        void adjust_lower(int move);
        int get_upper();
        int get_lower();
        int get_pin();
};