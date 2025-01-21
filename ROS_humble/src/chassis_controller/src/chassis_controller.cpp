/**
  **************************** 2024 Theseus Mecanum****************************
  * @file       chassis_controller.cpp/hpp
  * @brief      ROS humble package src for sbus px4 override control
  * @note       This node accepts the topic "chassis_command" in formation of 
  *             "vx%fvy%fz%f" and sends the sbus signal through serial port.
  *              when conneection down, pull over the vehicle automatically.
  *             Core functions :chassis_control_callback  main control function
  *                             uart_to_sbus              main transmission function
  *                             check_timeout             main watch dog function
  *             Core data struct: chassis_move_t          chassis control data struct
  *                               channels                static global sbus channels data struct
  *                               sbus_frame              static global sbus frame data struct 
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-21-2025     SPUTNIK         1. Alpha version
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **************************** COPYRIGHT 2025 Theseus Mecanum****************************
  */

#include "chassis_controller/chassis_controller.hpp"

class ChassisController : public rclcpp::Node
{
    // formate: x%fy%fz%f
public:
    ChassisController() : Node("chassis_controller"), ConnectionStatus(0), CommandStatus(0)
    {
        //串口初始化
        uart_init();


        //串口连接状态检查
        while (ConnectionStatus == 0)
        { // check if the serial is connected or wait for connection
            try
            {
                SbusPort.open(); // Open the serial port
                ConnectionStatus = 1;
                RCLCPP_INFO(this->get_logger(), "\033[42mUart connected.\033[0m"); // success
            }
            catch (const serial::IOException &e)
            {
                RCLCPP_INFO(this->get_logger(), "\033[41mUart not connected. Retrying...\033[0m"); // 串口无法打开
                std::this_thread::sleep_for(std::chrono::seconds(1));                              // fail, retry after 1s
            }
        }


        //订阅者设置
        motion_commander_ = this->create_subscription<std_msgs::msg::String>(
            "chassis_command", 10, [this](const std_msgs::msg::String::SharedPtr msg)  //话题位置
            { this->chassis_controll_callback(msg); });


        //看门狗计时器设置
        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&ChassisController::check_timeout, this));
        last_msg_time_ = this->now();
    }

private:

    // 运动控制订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_commander_; 

    // 看门狗计时器
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::Time last_msg_time_;

    // 串口连接状态和指令状态
    int ConnectionStatus; // 0 not connected; 1 OK
    int CommandStatus;    // 0 commands timeout; 1 OK

    // 速度设定，用于缓冲数据
    float vx_setpoint;
    float vy_setpoint;
    float wz_setpoint;
    float mode_setpoint;

    // 串口和底盘结构体
    serial::Serial SbusPort;
    std::shared_ptr<chassis_move_t> chassis_move;


    // 串口初始化
    void uart_init() // serial and chassis struct
    {
        SbusPort.setPort("/dev/ttyUSB0"); // serial initialization
        SbusPort.setBaudrate(115200);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // Timeout
        SbusPort.setTimeout(_time);

        //chassis结构体初始化
        chassis_move = std::make_shared<chassis_move_t>();
        chassis_move->mode = 0;
        chassis_move->vx_set = 0;
        chassis_move->vy_set = 0;
        chassis_move->wz_set = 0;
        chassis_move->throttle_debug = 0;
        channels[5] = PWM_MIN;  // Disarm the vehicle

        //速度设定初始化
        vx_setpoint = 0;
        vy_setpoint = 0;
        wz_setpoint = 0;
        mode_setpoint = 0;
    }


    // 看门狗计时器函数实现
    void check_timeout()
    {
        auto now = this->now();
        auto duration = now - last_msg_time_;

        using namespace std::chrono_literals;
        if (duration > rclcpp::Duration(0.5s))  // 0.5s内发生断连即停止
        {
            // Set velocities to zero
            chassis_move->vx_set = 0;
            chassis_move->vy_set = 0;
            chassis_move->wz_set = 0;
            chassis_move->mode = 0;

            channels[5] = PWM_MIN;  // Disarm the vehicle

            uart_to_sbus(); //transmit the signal
        }
        if (now - last_msg_time_ > rclcpp::Duration(6s)) {  // 5s内无指令即停止
            RCLCPP_WARN(this->get_logger(), "No chassis cmd signal. Stopped.");
            last_msg_time_ = now + rclcpp::Duration(1s);
        }
    }


    //核心控制函数
    void chassis_controll_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Chassis standingby");
        std::string motion_command = msg->data;
        
        // get vx vy wz from this frame
        // Use sscanf to extract vx, vy, wz values
        if (sscanf(motion_command.c_str(), "vx%fvy%fwz%f", &vx_setpoint, &vy_setpoint, &wz_setpoint) != 3)
        {
            std::cerr << "Invalid command format" << std::endl;
            return;
        }

        // Update chassis_move with new setpoints
        chassis_move->vx_set = vx_setpoint;
        chassis_move->vy_set = vy_setpoint;
        chassis_move->wz_set = wz_setpoint;
        chassis_move->mode = 0; // mode0
        channels[5] = PWM_MAX;  // arm the vehicle
        uart_to_sbus();

        RCLCPP_INFO(this->get_logger(), "chassis moving ad vx: %.2f, vy: %.2f, wz: %.2f", vx_setpoint, vy_setpoint, wz_setpoint);
        // Update last message time
        last_msg_time_ = this->now();
    }


    //核心发送函数
    void uart_to_sbus()
    {
        // motion
        speed_to_sbus(chassis_move.get(), channels);

        // define frame head
        sbus_frame[0] = SBUS_FRAME_HEAD;

        // define channels from byte 1 to byte 31
        for (int i = 0; i < 16; i++)
            sbus_frame[2 * i + 1] = channels[i] >> 8,
                               sbus_frame[2 * i + 2] = channels[i] & 0xff;

        // define frame tail
        sbus_frame[33] = SBUS_FLAGS;
        sbus_frame[34] = sbus_xor();

        try
        {
            SbusPort.write(sbus_frame, 35); // Sends data to the downloader via serial port
        }
        catch (serial::IOException &e)
        {
            std::cerr << "Unable to send data through serial port: " << e.what() << std::endl;
        }
    }

    void speed_to_sbus(chassis_move_t *chassis_move, uint16_t *channels)
    {

        // lateral speed to roll
        channels[0] = chassis_move->vy_set * 800 + 1000;
        // forward speed to pitch
        channels[1] = chassis_move->vx_set * 800 + 1000;
        // yaw speed to yaw
        channels[3] = chassis_move->wz_set * 800 + 1000;
        // thrrottle set to mid
        channels[2] = chassis_move->throttle_debug * 800 + 1000;
        // mode to ch5
        channels[4] = chassis_move->mode * 1000; // mode0 
        // ch6 for arming switch
        //channel 6 is under programm control, arm when chassis_cmd signal activated
        // others for none
        for (int i = 6; i < 16; i++)
        {
            channels[i] = PWM_MIN;
        }
    }


    //sbus辅助校验位函数
    uint8_t sbus_xor()
    {
        uint8_t xor_byte = 0;
        for (int i = 1; i <= 33; i++)
            xor_byte ^= sbus_frame[i];

        return xor_byte;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisController>());
    rclcpp::shutdown();
    return 0;
}