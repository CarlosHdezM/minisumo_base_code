#include <Arduino.h>
#include "robot_pin_definitions.h"
#include "motor_driver.hpp"
#include "line_sensor.hpp"
#include "robot_line_sensors_settings.h"
#include "opponent_sensor.hpp"
#include "minisumo_robot.h"


//Globals - Hardware resources.
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
    robot_pins::START_MODULE
};


//Globals - Global variables


void setup()
{
    robot.initialize();
    //Test Setup
    Serial.begin(115200);

    pinMode(robot_pins::SPARE_PIN, OUTPUT);
    digitalWrite(robot_pins::SPARE_PIN, LOW);
}


void loop()
{
    digitalWrite(robot_pins::SPARE_PIN, !digitalRead(robot_pins::SPARE_PIN));
    //volatile int val = read_robot_sensors();
    robot.motion(255,255,100,0b0,0b0);
    //Serial.println(val, BIN);
    //delay(50);
}

