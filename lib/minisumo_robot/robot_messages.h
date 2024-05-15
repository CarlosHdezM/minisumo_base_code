#ifndef ROBOT_MESSAGES_H
#define ROBOT_MESSAGES_H

#include "Arduino.h"

enum MotionEndReason : uint8_t {
    TIMEOUT           = 0x00,
    OPPONENT_DETECTED = 0x01,
    LINE_DETECTED     = 0x02,
    STOP_SIGNAL       = 0x04,
};


class SensorsStatus{
    public:
        //SensorsStatus(uint8_t opp, uint8_t line, bool start): opponent_sensors{opp}, line_sensors{line}, start_module{start}  {}
        uint8_t opponent_sensors;
        uint8_t line_sensors;
        bool start_module;
};


class MotionResult{
    public:
        MotionEndReason end_reason;
        SensorsStatus sensors;
};

#endif