#include <stdio.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include "chassis_controller.hpp"
#include "sbus.hpp"
#include "serial/serial.h"


int main(){

    serial::Serial SbusPort;

    SbusPort.setPort("COM5");

    SbusPort.setBaudrate(115200);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    SbusPort.setTimeout(_time);
    // try{
    //     SbusPort.open(); //Open the serial port //开启串口
    //     std::cout<< "Sbus connected" << std::endl;
    // } catch(){
    //     std::cerr << "Sbus offline!!" << e.what() << std::endl;
    // }

    chassis_controller mecanum = chassis_controller();

    clock_t start_time=clock();
	//std::thread thread1(uart_to_sbus);
    //thread1.detach();

    std::string cmd;

    while (std::cin >> cmd){
        if (cmd == "forward")
            mecanum.chassis_forward_control();
        else if (cmd == "rotate")
            mecanum.chassis_rotate_control();
        else if (cmd == "lat")
            mecanum.chassis_lat_control();
        else if (cmd == "circle")
            mecanum.chassis_circle_move();
        else if (cmd == "square")
            mecanum.chassis_square_move();
        else if (cmd == "stop")
            mecanum.chassis_stop();
        else
            printf("Invalid command.\n");

        //testing sbus
        uart_to_sbus();
    }
    
    mecanum.~chassis_controller();


    return 0;
}