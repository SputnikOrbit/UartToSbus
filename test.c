#include <stdio.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include "chassis_controller.hpp"
#include "sbus.hpp"
#include "serial/serial.h"

#include<csignal>

serial::Serial SbusPort;

chassis_controller mecanum = chassis_controller();

    SbusPort.setPort("COM7");

void signalHandler(int signum) {
    running = false;
}

void RT_uart_to_sbus() {
    auto last_time = std::chrono::high_resolution_clock::now();
    int loop_count = 0;

    while (true) {
        // 你的串口处理代码
        uart_to_sbus();
        //std::cout << "Uart to SBUS!" << std::endl;
        // 模拟处理时间
        loop_count++;
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - last_time;

        if (elapsed.count() >= 1.0) {
            std::cout << "RT_uart_to_sbus frequency: " << loop_count << " Hz" << std::endl;
            loop_count = 0;
            last_time = current_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(40)); // Sleep for 40 milliseconds to achieve 25Hz
    }
}

void RT_cmd_chassis() {

    std::string cmd;
    while (std::getline(std::cin, cmd)){
        if (cmd == "forward")
            mecanum.chassis_forward_control();
        else if (cmd == "backward")
            mecanum.chassis_backward_control();
        else if (cmd == "rotate")
            mecanum.chassis_rotate_control();
        else if (cmd == "unrotate")
            mecanum.chassis_unrotate_control();
        else if (cmd == "lat")
            mecanum.chassis_lat_control();
        else if (cmd == "unlat")
            mecanum.chassis_unlat_control();
        else if (cmd == "max")
            mecanum.throttle_max();
        else if (cmd == "idle")
            mecanum.throttle_idle();
        else if (cmd == "circle")
            mecanum.chassis_circle_move();
        else if (cmd == "square")
            mecanum.chassis_square_move();
        else if (cmd == "stop")
            mecanum.chassis_stop();
        else
            printf("Invalid command.\n");
    }
}


int main(){
    signal(SIGINT, signalHandler);

    std::cout << "Uart to SBUS!" << std::endl;

    

    SbusPort.setPort("/dev/ttyUSB0");

    SbusPort.setBaudrate(115200);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    SbusPort.setTimeout(_time);

    try {
        SbusPort.open(); // Open the serial port // 开启串口
        std::cout << "Sbus connected" << std::endl;
    } catch (const serial::IOException& e) {
        std::cerr << "Sbus offline!! " << e.what() << std::endl;
        return 1; // Exit the program if the serial port cannot be opened
    }

    chassis_controller mecanum = chassis_controller();

    clock_t start_time=clock();
	//std::thread thread1(uart_to_sbus);
    //thread1.detach();
    std::thread uart_thread(RT_uart_to_sbus);
    std::thread control_thread(RT_cmd_chassis);

    control_thread.join();
    uart_thread.join();
    
    //mecanum.~chassis_controller();


    return 0;
}