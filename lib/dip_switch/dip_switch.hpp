#ifndef ROBOT_DIP_SWITCH_H
#define ROBOT_DIP_SWITCH_H

#include "dip_switch.hpp"
#include "Arduino.h"

template <size_t NUM_PINS>
class DipSwitch
{
    public:
        DipSwitch(uint8_t pins[]) {
            for(uint8_t i = 0; i < NUM_PINS; i++){
                pins_[i] = pins[i];
            }
        }


        inline void initialize(){
            for(uint8_t i = 0; i < NUM_PINS; i++){
                pinMode(pins_[i], INPUT);
            }
        }


        inline uint8_t read(){
            uint8_t reading = 0;
            for(uint8_t i = 0; i < NUM_PINS; i++){
                reading |= digitalRead(pins_[i]) << i;
            }

            return reading;
        }

    private:
        uint8_t pins_[NUM_PINS];
};


#endif