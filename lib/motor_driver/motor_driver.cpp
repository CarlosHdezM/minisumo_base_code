#include "motor_driver.hpp"



void MotorDriver::initialize()
{
    pinMode(this->pin_1_, OUTPUT);
    pinMode(this->pin_2_, OUTPUT);
    digitalWrite(this->pin_1_, LOW);
    digitalWrite(this->pin_2_, LOW);
}


void MotorDriver::setVelocity(int16_t velocity) const //From -255 to 255 (8 bit resolution PWM)
{
    if(velocity > 0){
        digitalWrite(this->pin_1_, HIGH);
        analogWrite(this->pin_2_, 255-velocity);
    }
    else{
        analogWrite(this->pin_1_, 255+velocity);
        digitalWrite(this->pin_2_, HIGH);
    }
}