#ifndef _CHASSIS_CONTROLLER_H
    #define _CHASSIS_CONTROLLER_H

#include "serial/serial.h"
#include <regex>
#include <string>
#include <iostream>

#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PWM_MAX 1800
#define PWM_MIN 1000
#define PWM_MID 1500
#define PWM_OFF 500
#define PWM_OVER 2500

#define SBUS_FRAME_HEAD 0x0F
#define SBUS_FLAGS 0x00
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;


static uint8_t sbus_frame[35] = {0x00};

static uint16_t channels[16]; // 32 Bytes for 16 channels, Big endian


typedef struct
{
    unsigned short mode;    
    float vx_set;
    float vy_set;
    float wz_set;
    float throttle_debug;
} chassis_move_t;
/**
  * @brief          total 35 bytes per frame. 115200 8N1 
  * so max commu speed is 329.14 frames per sec. Pretty fast.
  * @param[in]      chassis_move: chassis move data struct
  * @param[out]     channels: remote control data struct point
  * @retval         none
  */
//void uart_to_sbus();

/**
  * @brief          trans from fp speed to std pwm 1000~2000
  * @param[in]      chassis_move: chassis move data struct
  * @param[out]     channels: remote control data struct point
  * @retval         none
  */
// void speed_to_sbus(chassis_move_t *chassis_move, uint16_t* channels);


/**
  * @brief          计算xor:为不包括帧头的所有数据异或
  * @param[in]      speed: speed set by user
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
// uint8_t sbus_xor(uint8_t *data, uint8_t len);

/**
  * @brief          打印检查数据帧
  * @retval         none
  */
// void print_frame(uint8_t *data, uint8_t len);

/**
  * @brief          打印检查通道pwm
  * @retval         none
  */
// void print_channels(uint16_t *data, uint8_t len);


#endif
