/**
  **************************** COPYRIGHT 2024 Theseus Mecanum****************************
  * @file       sbus.cpp/hpp
  * @brief      uart to sbus module programme
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
#include "chassis_controller.hpp"
#include "sbus.hpp"
#include <iostream>
#include <iomanip>
#include <serial/serial.h>

extern serial::Serial SbusPort;

void uart_to_sbus(){
    
    //motion 
    speed_to_sbus(&chassis_move, channels);

    std::cout << "sbus sent." << std::endl;

    //define frame head
    sbus_frame[0] = SBUS_FRAME_HEAD;

    //define channels from byte 1 to byte 31
for (int i = 0; i < 16; i++)
    sbus_frame[2 * i + 1] = channels[i] >> 8,
    sbus_frame[2 * i + 2] = channels[i] & 0xff;

    //define frame tail
    sbus_frame[33] = SBUS_FLAGS;
    sbus_frame[34] = sbus_xor(sbus_frame, 33);

    print_frame(sbus_frame, 35);
    print_channels(channels, 6);

//     try
//   {
//     Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
//   }
//   catch (serial::IOException& e)   
//   {
//     ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
//   } 
        try {
            SbusPort.write(sbus_frame, 35); // Sends data to the downloader via serial port
            std::cout << "sbus sent." << std::endl;
        } catch (serial::IOException& e) {
            std::cerr << "Unable to send data through serial port: " << e.what() << std::endl;
        }
}

/*辅助功能函数*/
void speed_to_sbus(chassis_move_t* chassis_move, uint16_t* channels){

    //lateral speed to roll
    channels[0] = chassis_move->vy_set * 800 + 1000;
    //forward speed to pitch
    channels[1] = chassis_move->vx_set * 800 + 1000;
    //yaw speed to yaw
    channels[3] = chassis_move->wz_set * 800 + 1000;
    //thrrottle set to mid
    channels[2] = chassis_move->throttle_debug * 800 + 1000;
    //mode to ch5
    channels[4] = (chassis_move->mode) * 500 + PWM_MID;
    //ch6 for arming switch
    channels[5] = PWM_MIN;
    //others for none
    for (int i = 6; i < 16; i++)
    {
        channels[i] = PWM_MIN;
    }
}


uint8_t sbus_xor(uint8_t *data, uint8_t len){
    uint8_t xor_byte = 0;
    for (int i = 1; i <= len; i++)
    {
        xor_byte ^= data[i];
    }
    return xor_byte;
}

void print_frame(uint8_t* frame, uint8_t length) {
    std::cout << "sbus_frame: ";
    for (size_t i = 0; i < length; i++) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)frame[i] << " ";
    }
    std::cout << std::endl;
}

void print_channels(uint16_t* channels, uint8_t length) {
    std::cout << "Channels PWM values:" << std::endl;
    for (size_t i = 0; i < length; i++) {
        std::cout << "Channel " << i << ": " << std::dec << channels[i] << std::endl;
    }
}
