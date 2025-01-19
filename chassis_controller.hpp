/**
  **************************** 2024 Theseus Mecanum****************************
  * @file       chassis_coltroller.cpp/hpp
  * @brief      Generic chassis controller ready for ros/ros2 integration
  *             
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
  **************************** 2024 Theseus Mecanum****************************
  */

 /*
 设计理念：包含底盘控制的vx, vy, wz速度和控制器（如串口）等状态的简单控制调试程序
 */

#ifndef _CHASSIS_CONTROLLER_HPP_
#define _CHASSIS_CONTROLLER_HPP_

//chassis motion mode define
#define YAW_LOCK_MODE 0
#define LAT_LOCK_MODE 1

//chassis motion limits
#define MAX_FORWARD_SPEED 1.0 // m/s
#define MAX_LATERAL_SPEED 1.0 // m/s
#define MAX_ROTATION_SPEED 1.0 // rad/s

typedef struct
{
    unsigned short mode;    
    float vx_set;
    float vy_set;
    float wz_set;
    float throttle_debug;
} chassis_move_t;

extern chassis_move_t chassis_move;


class chassis_controller
{
private:
    
public:
    chassis_controller();
    ~chassis_controller();

    void chassis_rotate_control();
    void chassis_unrotate_control();
    void chassis_backward_control();
    void chassis_forward_control();
    void chassis_lat_control();
    void chassis_unlat_control();   
    void throttle_max();
    void throttle_idle();
    void chassis_circle_move();
    void chassis_square_move();
    void chassis_stop();
};

#endif 
