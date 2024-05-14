#ifndef ROBOT_PIN_DEFINITIONS_H
#define ROBOT_PIN_DEFINITIONS_H

#include "Arduino.h"

namespace robot_pins
{
    //Sensors - Opponent detection
    constexpr uint8_t IR_SENSOR_L    = 2;
    constexpr uint8_t IR_SENSOR_LM   = A5;
    constexpr uint8_t IR_SENSOR_M    = A3;
    constexpr uint8_t IR_SENSOR_RM   = A1;
    constexpr uint8_t IR_SENSOR_R    = A0;
    
    //Sensors - Line detection
    constexpr uint8_t LINE_SENSOR_L  = A4;
    constexpr uint8_t LINE_SENSOR_R  = A2;
    
    //Actuators - Left motor driver
    constexpr uint8_t MOTOR_LEFT_1   = 6;
    constexpr uint8_t MOTOR_LEFT_2   = 5;
    
    //Actuators - Right motor driver
    constexpr uint8_t MOTOR_RIGHT_1  = 11;
    constexpr uint8_t MOTOR_RIGHT_2  = 10;
    
    //Selection - Dip switch
    constexpr uint8_t DIP_SW_BIT_0   = 7;
    constexpr uint8_t DIP_SW_BIT_1   = 8;
    constexpr uint8_t DIP_SW_BIT_2   = 9;
    
    //Start and stop
    constexpr uint8_t START_MODULE   = 3;
    
    //Miscellaneous
    constexpr uint8_t BUZZER         = 4;
    constexpr uint8_t USER_LED       = 13;
    constexpr uint8_t SPARE_PIN      = 12;
}


#endif