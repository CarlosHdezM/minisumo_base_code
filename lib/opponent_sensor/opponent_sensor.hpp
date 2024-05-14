#ifndef ROBOT_LINE_SENSOR_H
#define ROBOT_LINE_SENSOR_H

#include "Arduino.h"

class OpponentSensor
{
    public:
        OpponentSensor(uint8_t pin) : reading_pin_(pin){};
        inline void initialize() { pinMode(reading_pin_, INPUT); }
        inline uint16_t read() {return (digitalRead(this->reading_pin_)); }

    private:
        const uint8_t reading_pin_;
};

#endif