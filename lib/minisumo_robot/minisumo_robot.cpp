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
    uint8_t pin_start_module,
    DipSwitch<3> dip_switch,
    uint8_t pin_buzzer
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
    start_module_{pin_start_module},
    dip_switch_{dip_switch},
    pin_buzzer_{pin_buzzer}
{

}


void MinisumoRobot::initialize(){
    motor_left_.initialize();
    motor_right_.initialize();
    sensor_line_left_.initialize();
    sensor_line_right_.initialize();
    sensor_opp_l_.initialize();
    sensor_opp_lm_.initialize();
    sensor_opp_m_.initialize();
    sensor_opp_rm_.initialize();
    sensor_opp_r_.initialize();
    start_module_.initialize();
    dip_switch_.initialize();
}


const SensorsStatus & MinisumoRobot::read_sensors(){
    sensors_status_.opponent_sensors = 0B00000;
    sensors_status_.line_sensors = 0B00;
    //sensors_status_.start_module = 0; //"reseting" the start module variable is not needed  (no bitwise operations involved).
    sensors_status_.line_sensors = sensor_line_left_.read() << 1;
    sensors_status_.opponent_sensors |=
    sensor_opp_l_.read()        << 4 |
    sensor_opp_lm_.read()       << 3 |
    sensor_opp_m_.read()        << 2 |
    sensor_opp_rm_.read()       << 1 |
    sensor_opp_r_.read()        << 0 ;
    sensors_status_.line_sensors |= sensor_line_right_.read();
    sensors_status_.start_module = start_module_.read();
    return sensors_status_;
}


MotionResult MinisumoRobot::motion(int16_t left_wheel_vel, int16_t right_wheel_vel, 
    uint16_t max_time_ms, byte opp_sensors_mask, byte line_sensors_mask)
    {
        uint32_t _t_ini = millis();
        MotionResult motion_result;
        motion_result.sensors = this->read_sensors();
        motion_result.end_reason = MotionEndReason::TIMEOUT;
        //Set motors.
        motor_left_.setVelocity(left_wheel_vel);
        motor_right_.setVelocity(right_wheel_vel);
        //Keep reading until time expires or a sensor reading cancels the action.
        while(millis() - _t_ini < max_time_ms){
            motion_result.sensors = this->read_sensors();
            if (sensors_status_.start_module == 0){  //0 = Start module triggered "off" signal.
                motion_result.end_reason = MotionEndReason::STOP_SIGNAL;
                break;
            }
            if (sensors_status_.line_sensors & line_sensors_mask){  //Bitwise operation, checking individual bits.
                motion_result.end_reason = MotionEndReason::LINE_DETECTED;
                break;
            }
            if (sensors_status_.opponent_sensors & opp_sensors_mask){ //Bitwise operation, checking individual bits.
                motion_result.end_reason = MotionEndReason::OPPONENT_DETECTED;
                break;
            }
        }

        return motion_result;
    }
