#include "state_machine.h"
#include "Arduino.h"
#include "minisumo_robot.h"
//DEBUG:
#include "robot_pin_definitions.h"
#include "robot_line_sensors_settings.h"

namespace RobotSM{

    function_choice state_machine_functions[State::STATES_COUNT] = {stopped, retreat, searching, pre_attack, attack};
    function_choice search_functions[SEARCH_FCTS_COUNT] = {search_0, search_1, search_2, search_3};
    function_choice pre_attack_functions[PRE_ATTACK_FCTS_COUNT] = {pre_attack_0, pre_attack_1};
    uint8_t search_fct_idx = 0;
    uint8_t pre_attack_fct_idx = 0;


    State run_transition(State next_state, MinisumoRobot& robot){
        return state_machine_functions[next_state](robot);
    }



    /*
    *Select searching method from dip switch.
    *Select pre_attack method from dip switch.
    *Check for start module. break (transition) if found.
    *Return next state.
    */
    State stopped(MinisumoRobot& robot){
        robot.motion(0,0,0,0,0); //STOP ROBOT.
        while(1){
            byte dip_sw_status = robot.read_dip_sw();
            search_fct_idx = dip_sw_status & ~(1 << PRE_ATTACK_DIP_SW_BIT); //Clear(ignore) the dip switch pre_attack bit. Clearing is done on the temporary, not dip_sw_status.
            pre_attack_fct_idx = (dip_sw_status >> PRE_ATTACK_DIP_SW_BIT) & 1; //Get the value of the dip switch pre_attack bit.
            bool start_module_status = robot.read_sensors().start_module;
            //DEBUG INIT
            //Serial.print("Stopped State! \n");
            //DEBUG END
            if (start_module_status){
                break;
            }
        }
        return RobotSM::State::SEARCHING;
    }


    //Basic retreat function.
    //ToDo: Implement alternative retreat. Implement a stack to push and pop line sensor status (detect if we are inside or outside the dohyo)
    State retreat(MinisumoRobot& robot){
        robot.setMotors(0, 0); //Stop the robot.
        byte line_arrival_status = robot.get_last_sensor_readings().line_sensors;
        //Go back at full speed for X milliseconds (take the robot back to the dohyo in case that the inertia took it beyond).
        robot.motion(-255,-255,200,0b00000, 0b00);   //!! As both sensors masks are 0, this motion becomes blocking. We are "blind" for 200 ms.
        //ToDo: could we still be beyond the white line here? Implement a more robust function with a stack. Push and pop line readings.
        //At this point, we assume we are inside the dohyo (None of the QTRs is beyond the white line).
        //If both line sensors are seeing white - Need to go back more first. A default timeout is considered to prevent the robot from moving back indefinetely.
        for (uint32_t init_time_go_back = millis(); (millis() - init_time_go_back) < 750 and (robot.read_sensors().line_sensors == 0B11) ; ){
            //No action needed. Sensor refreshing inside the loop condition.
        }
        //At least one line sensor is seeing black - No need to go back more. We can directly turn accordingly.
        //Turn according to the line sensor readings when arrived to the line.
        MotionResult motion_result;
        if (line_arrival_status & 0B10){        //Arrived with left line sensor. Turn right.
            motion_result = robot.motion(255, -255, line_sensor_settings::TIME_TURN_WHITE_LINE, 0B10001, 0B00);
        }
        else {                                  //Arrived with right line sensor or both. Turn right.
            motion_result = robot.motion(-255, 255, line_sensor_settings::TIME_TURN_WHITE_LINE, 0B10001, 0B00);
        }
        //Transition to next state.
        RobotSM::State next_state;
        if (motion_result.end_reason == MotionEndReason::STOP_SIGNAL){
            next_state = RobotSM::State::STOPPED;
        }
        else {
            next_state = RobotSM::State::ATTACKING;
        }
        return next_state;
    }


    State searching(MinisumoRobot& robot){
        return search_functions[search_fct_idx](robot);
    }


    State search_0(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        MotionMessage motions[]{
            {255, 255, 100, 0b11111, 0b11}, 
            {0, 0, 2000, 0b11111, 0b11}
        };
        size_t motions_count = sizeof(motions) / sizeof(*motions);
        for(uint8_t curr_search_step = 0; true ;curr_search_step = (curr_search_step + 1) % motions_count){ //Infinite void loop (induced rollover)
            //Serial.print("Search 0! \t");
            MotionResult motion_result = robot.motion(motions[curr_search_step]);
            
            //Transition conditions
            if (motion_result.end_reason == MotionEndReason::STOP_SIGNAL){
                next_state = RobotSM::State::STOPPED;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::LINE_DETECTED){
                next_state = RobotSM::State::RETREAT;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::OPPONENT_DETECTED){
                next_state = RobotSM::State::PRE_ATTACKING;
                break;
            }
        }
        return next_state;
    }

    //Search 1: Searching moving straight at full speed and bouncing on the line.
    State search_1(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        while(1){
            //Serial.print("Search 1! \t");
            MotionResult motion_result = robot.motion(255,255,150,0b11111, 0b11);
            
            //Transition conditions
            if (motion_result.end_reason == MotionEndReason::STOP_SIGNAL){
                next_state = RobotSM::State::STOPPED;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::LINE_DETECTED){
                next_state = RobotSM::State::RETREAT;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::OPPONENT_DETECTED){
                next_state = RobotSM::State::PRE_ATTACKING;
                break;
            }
        }
        return next_state;
    }


    State search_2(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        MotionMessage motions[]{
            {255, 255, 100, 0b11111, 0b11}, 
            {0, 0, 100, 0b11111, 0b11},
            {255, 255, 100, 0b11111, 0b11},
            {0, 0, 100, 0b11111, 0b11},
            {255, 255, 100, 0b11111, 0b11},
            {0, 0, 100, 0b11111, 0b11},
            {255, -255, 70, 0b11111, 0b11},
            {0, 0, 100, 0b11111, 0b11},
            {-255, -255, 100, 0b11111, 0b11}, 
            {0, 0, 100, 0b11111, 0b11},
            {-255, -255, 100, 0b11111, 0b11}, 
            {0, 0, 100, 0b11111, 0b11},
            {-255, -255, 100, 0b11111, 0b11}, 
            {0, 0, 2000, 0b11111, 0b11},
        };
        size_t motions_count = sizeof(motions) / sizeof(*motions);
        for(uint8_t curr_search_step = 0; true ;curr_search_step = (curr_search_step + 1) % motions_count){ //Infinite void loop (induced rollover)
            //Serial.print("Search 2! \t");
            MotionResult motion_result = robot.motion(motions[curr_search_step]);
            
            //Transition conditions
            if (motion_result.end_reason == MotionEndReason::STOP_SIGNAL){
                next_state = RobotSM::State::STOPPED;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::LINE_DETECTED){
                next_state = RobotSM::State::RETREAT;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::OPPONENT_DETECTED){
                next_state = RobotSM::State::PRE_ATTACKING;
                break;
            }
        }
        return next_state;
    }


    State search_3(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        while(1){
            Serial.print("Search 3! \t");
        }
        return next_state;
    }


    State pre_attack(MinisumoRobot& robot){
        return RobotSM::State::ATTACKING;  //!! Remove this line. Bypass pre_attack for now for debugging purposes. 
        return pre_attack_functions[pre_attack_fct_idx](robot);
    }


    State pre_attack_0(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::PRE_ATTACKING; //Default next transition -> Itself.
        while(1){
            Serial.print("PRE ATTACK 0! \t");
        }
        return next_state;
    }


    State pre_attack_1(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::PRE_ATTACKING; //Default next transition -> Itself.
        while(1){
            Serial.print("PRE ATTACK 1! \t");
        }
        return next_state;
    }


    State attack(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::ATTACKING; //Default next transition -> Itself.
        while(1){ 
            SensorsStatus robot_sensors = robot.read_sensors();
            MotionResult motion_result;
            //Opponent in front.
            if (robot_sensors.opponent_sensors & 0B00100){
                motion_result = robot.motion(255, 255, 200, 0B11011, 0B11);
            }
            //Opponent to the left.
            else if (robot_sensors.opponent_sensors & 0B11000){
                motion_result = robot.motion(-255, 255, 200, 0B00111, 0B11);
            }
            //Opponent to the right.
            else if (robot_sensors.opponent_sensors & 0B00011){
                motion_result = robot.motion(255, -255, 200, 0B11100, 0B11);
            }
            //No opponent detected: Give a threshold period to try to detect before transitioning to searching.
            else {
                motion_result = robot.motion(0,0,100,0B11111, 0B11); 
            }
            //Machine State transitions:
            //Transition conditions
            if (motion_result.end_reason == MotionEndReason::STOP_SIGNAL){
                next_state = RobotSM::State::STOPPED;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::LINE_DETECTED){
                next_state = RobotSM::State::RETREAT;
                break;
            }
            if (motion_result.end_reason == MotionEndReason::OPPONENT_DETECTED){
                //next_state = RobotSM::State::ATTACKING;   //Stay attacking.
                continue;
            }
            if (motion_result.end_reason == MotionEndReason::TIMEOUT){
                next_state = RobotSM::State::SEARCHING;
                break;
            }

        }
        return next_state;
    }






}