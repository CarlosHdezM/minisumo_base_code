#include "minisumo_robot.h"


MinisumoRobot::MinisumoRobot(
    uint8_t pin_1_motor_left, uint8_t pin_2_motor_left,
    uint8_t pin_1_motor_right, uint8_t pin_2_motor_right,
    uint8_t pin_sensor_line_l, uint16_t threshold_sensor_line_l,
    uint8_t pin_sensor_line_r, uint16_t threshold_sensor_line_r,
    uint8_t pin_opp_sensor_l,
    uint8_t pin_opp_sensor_lm,
    uint8_t pin_opp_sensor_m,
    uint8_t pin_opp_sensor_rm,
    uint8_t pin_opp_sensor_r,
    uint8_t pin_start_module
    ):
    motor_left_{pin_1_motor_left, pin_2_motor_left},
    motor_right_{pin_1_motor_right, pin_2_motor_right},
    sensor_line_left_{pin_sensor_line_l, threshold_sensor_line_l},
    sensor_line_right_{pin_sensor_line_r, threshold_sensor_line_r},
    sensor_opp_l_{pin_opp_sensor_l},
    sensor_opp_lm_{pin_opp_sensor_lm},
    sensor_opp_m_{pin_opp_sensor_m},
    sensor_opp_rm_{pin_opp_sensor_rm},
    sensor_opp_r_{pin_opp_sensor_r},
    start_module_{pin_start_module}
    {

    }


    byte MinisumoRobot::read_sensors(){
        byte sensor_readings = 0B00000000;
        sensor_readings |=
        sensor_line_left_.read()    << 2 |   
        sensor_opp_l_.read()        << 7 |
        sensor_opp_lm_.read()       << 6 |
        sensor_opp_m_.read()        << 5 |
        sensor_opp_rm_.read()       << 4 |
        sensor_opp_r_.read()        << 3 |
        sensor_line_right_.read()   << 1 |
        start_module_.read()        << 0;
        return sensor_readings;
    }