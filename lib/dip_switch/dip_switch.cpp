/*#include "dip_switch.hpp"

template <size_t NUM_PINS>
DipSwitch<NUM_PINS>::DipSwitch(uint8_t pins[])
{
    for(uint8_t i = 0; i < NUM_PINS; i++){
        pins_[i](pins[i]);
    }
}


template <size_t NUM_PINS>
void DipSwitch<NUM_PINS>::initialize(){
    for(uint8_t i = 0; i < NUM_PINS; i++){
        pinMode(pins_[i], INPUT);
    }
}


template <size_t NUM_PINS>
uint8_t DipSwitch<NUM_PINS>::read(){
    uint8_t reading = 0;
    for(uint8_t i = 0; i < NUM_PINS; i++){
        reading |= digitalRead(pins_[i]) << i;
    }

    return reading;
}



*/