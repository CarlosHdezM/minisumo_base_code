#ifndef ROBOT_MOTOR_DRIVER_H
#define ROBOT_MOTOR_DRIVER_H

#include "Arduino.h"

class MotorDriver
{
    public:
        MotorDriver(uint8_t pin_1, uint8_t pin_2) : pin_1_(pin_1), pin_2_(pin_2){};
        void initialize();
        void setVelocity(int16_t velocity) const; // From -255 to 255 (8 bit resolution PWM)

    private:
        const uint8_t pin_1_;
        const uint8_t pin_2_;
};

#endif