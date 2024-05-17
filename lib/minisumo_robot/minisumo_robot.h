#ifndef ROBOT_MINISUMO_ROBOT_H
#define ROBOT_MINISUMO_ROBOT_H

#include "Arduino.h"
#include "opponent_sensor.hpp"
#include "line_sensor.hpp"
#include "motor_driver.hpp"
#include "dip_switch.hpp"
#include "robot_messages.h"

//The robot start module is a simple digital input, same as the opponent sensors. "Aliasing" the class. 
typedef OpponentSensor StartModule;

#define DIP_SWITCH_BITS 3

class MinisumoRobot{
    public:
        //Public member functions
        MinisumoRobot(
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
            DipSwitch<DIP_SWITCH_BITS> dip_switch,
            uint8_t pin_buzzer
            );
        void initialize();
        MotionResult motion(const MotionMessage & motion_message);
        MotionResult motion(
            int16_t left_wheel_vel, int16_t right_wheel_vel, 
            uint16_t max_time_ms, byte opp_sensors_mask, byte line_sensors_mask);
        inline void setMotors(int16_t left_wheel_vel, int16_t right_wheel_vel) const{
                motor_left_.setVelocity(left_wheel_vel); 
                motor_right_.setVelocity(right_wheel_vel);
        }
        const SensorsStatus & read_sensors();
        inline const SensorsStatus & get_last_sensor_readings() const{return sensors_status_;}
        byte read_dip_sw(){return dip_switch_.read();}


    private:
        //Private data members
        MotorDriver motor_left_;
        MotorDriver motor_right_;
        LineSensor sensor_line_left_;
        LineSensor sensor_line_right_;
        OpponentSensor sensor_opp_l_;
        OpponentSensor sensor_opp_lm_;
        OpponentSensor sensor_opp_m_;
        OpponentSensor sensor_opp_rm_;
        OpponentSensor sensor_opp_r_;
        StartModule start_module_;
        DipSwitch<DIP_SWITCH_BITS> dip_switch_;
        uint8_t pin_buzzer_;
        SensorsStatus sensors_status_;


        //Private member functions
        
        
};
#endif
