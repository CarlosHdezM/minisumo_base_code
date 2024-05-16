#ifndef STATE_MACHINE_ROBOT_H
#define STATE_MACHINE_ROBOT_H

#include "stdint.h"
#include "minisumo_robot.h"

namespace RobotSM{

    enum class State : uint8_t{
        STOPPED, 
        RETREAT,
        SEARCHING,
        PRE_ATTACKING,
        ATTACKING,
    };

    State stopped(MinisumoRobot& robot);
    State retreat(MinisumoRobot& robot);
    State searching(MinisumoRobot& robot);
    State pre_attack(MinisumoRobot& robot);
    State attack(MinisumoRobot& robot);

    //Searching methods.
    //ToDo: Replace the names to descriptive ones (behaviour is to define yet).
    typedef State (*function_choice)(MinisumoRobot&); //Typedef of a new function pointer type: returns State, takes in MinisumoRobot &.
    #define SEARCH_FCTS_COUNT 4
    State search_0(MinisumoRobot& robot);
    State search_1(MinisumoRobot& robot);
    State search_2(MinisumoRobot& robot);
    State search_3(MinisumoRobot& robot);

    #define PRE_ATTACK_FCTS_COUNT 2
    #define PRE_ATTACK_DIP_SW_BIT 2
    State pre_attack_0(MinisumoRobot& robot);
    State pre_attack_1(MinisumoRobot& robot);



}


#endif