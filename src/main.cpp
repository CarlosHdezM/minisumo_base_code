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


void setup()
{
    //Test Setup
    Serial.begin(115200);
    robot.initialize();

    //Spare pin is used for debugging purposes (logic analyzer connected to this pin)
    pinMode(robot_pins::SPARE_PIN, OUTPUT);
    digitalWrite(robot_pins::SPARE_PIN, LOW);
}



void loop()
{
    current_state = RobotSM::run_transition(current_state, robot);

    // Debug init
    digitalWrite(robot_pins::SPARE_PIN, !digitalRead(robot_pins::SPARE_PIN));
    Serial.println("DEBUG: Back to loop");
    // Debug end
}
