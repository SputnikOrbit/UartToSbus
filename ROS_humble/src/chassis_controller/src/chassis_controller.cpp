
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "chassis_controller/chassis_controller.hpp"


class ChassisController : public rclcpp::Node
{
    //formate: x%fy%fz%f
    public:
        ChassisController():
            Node("chassis_controller"), ConnectionStatus(0), CommandStatus(0)
        {
            uart_init();

            while (ConnectionStatus == 0) { //check if the serial is connected or wait for connection
                try {
                    SbusPort.open(); // Open the serial port 
                    ConnectionStatus = 1;
                    RCLCPP_INFO(this->get_logger(), "\033[42mUart connected.\033[0m"); //success
                } catch (const serial::IOException &e) {
                    RCLCPP_INFO(this->get_logger(), "\033[41mUart not connected. Retrying...\033[0m"); //串口无法打开
                    std::this_thread::sleep_for(std::chrono::seconds(1)); // fail, retry after 1s
                }
            }

            //motion_commander_ = this->create_subscription<std_msgs::msg::String>(
                //"motion_commands", 10, std::bind(&ChassisController::chassis_controll_callback, this, std::placeholders::_1));
            motion_commander_ = this->create_subscription<std_msgs::msg::String>(
    "motion_commands", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        this->chassis_controll_callback(msg);
    });
        }

    private:
        void uart_init(){
            SbusPort.setPort("/dev/ttyUSB0"); //串口初始化
            SbusPort.setBaudrate(115200);
            serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout 
            SbusPort.setTimeout(_time);
        }

        void chassis_controll_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Chassis standingby");
            this->uart_to_sbus();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        void uart_to_sbus(){

        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_commander_; //运动控制订阅者
        rclcpp::TimerBase::SharedPtr timer_;
        int ConnectionStatus; //0 not connected; 1 OK 
        int CommandStatus; //0 commands timeout; 1 OK
        serial::Serial SbusPort;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisController>());
    rclcpp::shutdown();
    return 0;
}