#ifndef ROBOT_LINE_SENSOR_H
#define ROBOT_LINE_SENSOR_H

#include "Arduino.h"

class LineSensor
{
    public:
        LineSensor(uint8_t pin, uint16_t threshold_white) : reading_pin_(pin), threshold_white_(threshold_white){};
        inline void initialize() { pinMode(reading_pin_, INPUT); }
        inline bool read() { return (analogRead(this->reading_pin_) < this->threshold_white_) ? 1 : 0; }
        inline uint16_t read_RAW() {return (analogRead(this->reading_pin_)); }

    private:
        const uint8_t reading_pin_;
        const int threshold_white_; //NOTE: int type instead of uint16_t to avoid  -Wsign-compare warning (analogRead returns int).
};

#endif