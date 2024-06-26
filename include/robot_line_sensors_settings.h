#ifndef ROBOT_LINE_SENSORS_SETTINGS_H
#define ROBOT_LINE_SENSORS_SETTINGS_H
#include "stdint.h"

namespace line_sensor_settings{
    constexpr uint16_t left_threshold_white{350};
    constexpr uint16_t right_threshold_white{350};
    constexpr uint16_t TIME_TURN_WHITE_LINE{160};
}

#endif