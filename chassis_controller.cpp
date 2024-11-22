/**
  **************************** 2024 Theseus Mecanum****************************
  * @file       chassis_coltroller.cpp/hpp
  * @brief      Generic chassis controller ready for ros/ros2 integration
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-26-2024     SPUTNIK         1. constructing
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **************************** COPYRIGHT 2024 Theseus Mecanum****************************
  */

#include <stdio.h>
#include "chassis_controller.hpp"
#include "sbus.hpp"
#include <iostream>

chassis_move_t chassis_move;

chassis_controller::chassis_controller(){
    printf("Chassis init. Waiting for commands\n");
    chassis_move.mode = YAW_LOCK_MODE;
    chassis_move.vx_set = 0;
    chassis_move.vy_set = 0;
    chassis_move.wz_set = 0;
    
}

chassis_controller::~chassis_controller(){
    chassis_move.mode = YAW_LOCK_MODE;
    chassis_move.vx_set = 0;
    chassis_move.vy_set = 0;
    chassis_move.wz_set = 0;
    printf("Chassis controller destroyed.\n");
    
}

void chassis_controller::chassis_stop() {
    printf("\033[31mStopping the mec1.\033[0m\n"); // 红色
    chassis_move.mode = YAW_LOCK_MODE;
    chassis_move.vx_set = 0;
    chassis_move.vy_set = 0;
    chassis_move.wz_set = 0;
}

void chassis_controller::chassis_forward_control() {
    printf("\033[32mForward moving.\033[0m\n"); // 绿色
    chassis_move.mode = YAW_LOCK_MODE;
    chassis_move.vx_set = 1.0;
    chassis_move.vy_set = 0;
    chassis_move.wz_set = 0;
}

void chassis_controller::chassis_rotate_control() {
    printf("\033[34mRotating.\033[0m\n"); // 蓝色
    chassis_move.mode = LAT_LOCK_MODE;
    chassis_move.vx_set = 0;
    chassis_move.vy_set = 0;
    chassis_move.wz_set = 1.0;
}

void chassis_controller::chassis_lat_control() {
    printf("\033[33mLateral moving.\033[0m\n"); // 黄色
    chassis_move.mode = YAW_LOCK_MODE;
    chassis_move.vx_set = 0;
    chassis_move.vy_set = 1.0;
}

void chassis_controller::chassis_circle_move(){
    printf("Circle moving.\n");
}

void chassis_controller::chassis_square_move(){
    printf("Square moving.\n");
}


