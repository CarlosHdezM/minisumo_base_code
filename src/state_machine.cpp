#include "state_machine.h"
#include "Arduino.h"
#include "minisumo_robot.h"
//DEBUG:
#include "robot_pin_definitions.h"

namespace RobotSM{

    function_choice search_functions[SEARCH_FCTS_COUNT] = {search_0, search_1, search_2, search_3};
    uint8_t search_fct_idx = 0;
    function_choice pre_attack_functions[PRE_ATTACK_FCTS_COUNT] = {pre_attack_0, pre_attack_1};
    uint8_t pre_attack_fct_idx = 0;

    /*
    *Wait for start module.
    *Select searching method from dip switch.
    *Select pre_attack method from dip switch.
    *Return next state.
    */
    State stopped(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::STOPPED; //Default next transition -> Itself.
        while(1){
            byte dip_sw_status = robot.read_dip_sw();
            search_fct_idx = dip_sw_status & ~(1 << PRE_ATTACK_DIP_SW_BIT); //Clear(ignore) the dip switch pre_attack bit. Clearing is done on the temporary, not dip_sw_status.
            pre_attack_fct_idx = (dip_sw_status >> PRE_ATTACK_DIP_SW_BIT) & 1; //Get the value of the dip switch pre_attack bit.
            //DEBUG INIT
            Serial.print("Stopped State! \n");
            Serial.print("search function index: ");
            Serial.println(search_fct_idx);
            next_state = RobotSM::State::PRE_ATTACKING;
            break;
            //DEBUG END
        }
        return next_state;
    }


    State retreat(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::RETREAT; //Default next transition -> Itself.
        while(1){
            Serial.print("Retreating! \t");
        }
        return next_state;
    }


    State searching(MinisumoRobot& robot){
        return search_functions[search_fct_idx](robot);
    }


    State search_0(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        while(1){
            Serial.print("Search 0! \t");
        }
        return next_state;
    }


    State search_1(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        while(1){
            Serial.print("Search 1! \t");
        }
        return next_state;
    }


    State search_2(MinisumoRobot& robot){
        RobotSM::State next_state = RobotSM::State::SEARCHING; //Default next transition -> Itself.
        while(1){
            Serial.print("Search 2! \t");
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
            Serial.print("Attacking! \t");
        }
        return next_state;
    }






}