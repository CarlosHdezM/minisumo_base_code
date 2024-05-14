#include <Arduino.h>
#include "robot_pin_definitions.h"
#include "motor_driver.hpp"
#include "line_sensor.hpp"
#include "robot_line_sensors_settings.h"

MotorDriver motor_left{robot_pins::MOTOR_LEFT_1, robot_pins::MOTOR_LEFT_2};
MotorDriver motor_right{robot_pins::MOTOR_RIGHT_1, robot_pins::MOTOR_RIGHT_2};

LineSensor left_line_sensor{robot_pins::LINE_SENSOR_L, line_sensor_settings::left_threshold_white};
LineSensor right_line_sensor{robot_pins::LINE_SENSOR_R, line_sensor_settings::right_threshold_white};

byte read_robot_sensors();

void setup()
{
    motor_left.initialize();
    motor_right.initialize();

    // TEST - PRINT
    Serial.begin(115200);

    // TEST - SETUP PINS FOR SENSORS
    pinMode(robot_pins::IR_SENSOR_L, INPUT_PULLUP);
    pinMode(robot_pins::IR_SENSOR_LM, INPUT_PULLUP);
    pinMode(robot_pins::IR_SENSOR_M, INPUT_PULLUP);
    pinMode(robot_pins::IR_SENSOR_RM, INPUT_PULLUP);
    pinMode(robot_pins::IR_SENSOR_R, INPUT_PULLUP);
    left_line_sensor.initialize();
    right_line_sensor.initialize();

    pinMode(robot_pins::SPARE_PIN, OUTPUT);
    digitalWrite(robot_pins::SPARE_PIN, LOW);
}


void loop()
{
    digitalWrite(robot_pins::SPARE_PIN, !digitalRead(robot_pins::SPARE_PIN));
    volatile int val = read_robot_sensors();
    //Serial.println(val, BIN);
    //delay(50);
}


byte read_robot_sensors()
{
    byte sensor_readings = 0B00000000;
    sensor_readings |=
        left_line_sensor.read()  << 2 |
        digitalRead(robot_pins::IR_SENSOR_L)  << 7 |
        digitalRead(robot_pins::IR_SENSOR_LM) << 6 |
        digitalRead(robot_pins::IR_SENSOR_M)  << 5 |
        digitalRead(robot_pins::IR_SENSOR_RM) << 4 |
        digitalRead(robot_pins::IR_SENSOR_R)  << 3 |
        right_line_sensor.read() << 1 |
        digitalRead(robot_pins::START_MODULE) << 0;

    return sensor_readings;
}
