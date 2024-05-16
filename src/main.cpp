#include <Arduino.h>
#include "robot_pin_definitions.h"
#include "motor_driver.hpp"
#include "line_sensor.hpp"
#include "robot_line_sensors_settings.h"
#include "opponent_sensor.hpp"
#include "minisumo_robot.h"
#include "dip_switch.hpp"
#include "state_machine.h"


//Globals - Hardware resources.
uint8_t DIP_SW[] = {robot_pins::DIP_SW_BIT_0, robot_pins::DIP_SW_BIT_1, robot_pins::DIP_SW_BIT_2};
MinisumoRobot robot{
    robot_pins::MOTOR_LEFT_1, robot_pins::MOTOR_LEFT_2,
    robot_pins::MOTOR_RIGHT_1, robot_pins::MOTOR_RIGHT_2,
    robot_pins::LINE_SENSOR_L, line_sensor_settings::left_threshold_white,
    robot_pins::LINE_SENSOR_R, line_sensor_settings::right_threshold_white,
    robot_pins::IR_SENSOR_L,
    robot_pins::IR_SENSOR_LM,
    robot_pins::IR_SENSOR_M,
    robot_pins::IR_SENSOR_RM,
    robot_pins::IR_SENSOR_R,
    robot_pins::START_MODULE,
    DipSwitch<robot_pins::DIP_SW_NUM_BITS>(DIP_SW),
    robot_pins::BUZZER
};

RobotSM::State current_state{RobotSM::State::STOPPED};


//Globals - Global variables


void setup()
{
    //Test Setup
    Serial.begin(115200);
    robot.initialize();

    pinMode(robot_pins::SPARE_PIN, OUTPUT);
    digitalWrite(robot_pins::SPARE_PIN, LOW);
}



void loop()
{
    
    switch (current_state)
    {
        case RobotSM::State::STOPPED:
            current_state = RobotSM::stopped(robot);
            break;

        case RobotSM::State::RETREAT:
            current_state = RobotSM::retreat(robot);
            break;

        case RobotSM::State::SEARCHING:
            current_state = RobotSM::searching(robot);
            break;

        case RobotSM::State::PRE_ATTACKING:
            current_state = RobotSM::pre_attack(robot);
            break;

        case RobotSM::State::ATTACKING:
            current_state = RobotSM::attack(robot);
            break;


        default:
            Serial.print("ERROR: WE SHOULDN'T REACH THIS CASE.");
            break;
    }


    digitalWrite(robot_pins::SPARE_PIN, !digitalRead(robot_pins::SPARE_PIN));
    Serial.println("HEY");
    //volatile int val = robot.();
    //robot.motion(255,255,100,0b0,0b0);
    //Serial.println(val, BIN);
    //volatile uint8_t val2 = robot.read_dip_sw();
    //auto response{robot.read_sensors()};
    //volatile auto result = robot.motion(255,255,0,0b0,0b0);
    //volatile auto result = robot.read_sensors();
    //response.line_sensors = 3;
    //Serial.print(response.line_sensors);
    //Serial.print("\t");
    //Serial.print(response.opponent_sensors);
    //Serial.print("\t");
    //Serial.print(response.start_module);
    //Serial.println(val2);
   // Serial.println();
}
