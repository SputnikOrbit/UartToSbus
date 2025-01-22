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

#endif