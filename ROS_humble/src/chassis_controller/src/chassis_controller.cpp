#include "chassis_controller/chassis_controller.hpp"

class ChassisController : public rclcpp::Node
{
    // formate: x%fy%fz%f
public:
    ChassisController() : Node("chassis_controller"), ConnectionStatus(0), CommandStatus(0)
    {
        uart_init();

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

        // motion_commander_ = this->create_subscription<std_msgs::msg::String>(
        //"motion_commands", 10, std::bind(&ChassisController::chassis_controll_callback, this, std::placeholders::_1));
        motion_commander_ = this->create_subscription<std_msgs::msg::String>(
            "chassis_command", 10, [this](const std_msgs::msg::String::SharedPtr msg)
            { this->chassis_controll_callback(msg); });

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&ChassisController::check_timeout, this));

        // Initialize last message time
        last_msg_time_ = this->now();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_commander_; // 运动控制订阅者
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_msg_time_;
    int last_log_time_;
    int ConnectionStatus; // 0 not connected; 1 OK
    int CommandStatus;    // 0 commands timeout; 1 OK
    float vx_setpoint;
    float vy_setpoint;
    float wz_setpoint;
    float mode_setpoint;
    serial::Serial SbusPort;
    std::shared_ptr<chassis_move_t> chassis_move;

    void uart_init() // serial and chassis struct
    {
        SbusPort.setPort("/dev/ttyUSB0"); // serial initialization
        SbusPort.setBaudrate(115200);
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); // Timeout
        SbusPort.setTimeout(_time);
        chassis_move = std::make_shared<chassis_move_t>();
        chassis_move->mode = 0;
        chassis_move->vx_set = 0;
        chassis_move->vy_set = 0;
        chassis_move->wz_set = 0;
        chassis_move->throttle_debug = 0;

        vx_setpoint = 0;
        vy_setpoint = 0;
        wz_setpoint = 0;
        mode_setpoint = 0;
    }

    void check_timeout()
    {
        auto now = this->now();
        auto duration = now - last_msg_time_;

        using namespace std::chrono_literals;
        if (duration > rclcpp::Duration(0.5s))
        {
            // Set velocities to zero
            chassis_move->vx_set = 0;
            chassis_move->vy_set = 0;
            chassis_move->wz_set = 0;
            uart_to_sbus();
        }
        if (now - last_msg_time_ > rclcpp::Duration(6s)) {
            RCLCPP_WARN(this->get_logger(), "No chassis cmd signal. Stopped.");
            last_msg_time_ = now + rclcpp::Duration(1s);
        }
    }

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
        uart_to_sbus();

        RCLCPP_INFO(this->get_logger(), "chassis moving ad vx: %.2f, vy: %.2f, wz: %.2f", vx_setpoint, vy_setpoint, wz_setpoint);
        // Update last message time
        last_msg_time_ = this->now();
    }

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
        channels[4] = (chassis_move->mode) * 500 + PWM_MID;
        // ch6 for arming switch
        channels[5] = PWM_MIN;
        // others for none
        for (int i = 6; i < 16; i++)
        {
            channels[i] = PWM_MIN;
        }
    }

    uint8_t sbus_xor()
    {
        uint8_t xor_byte = 0;
        for (int i = 1; i <= 33; i++)
        {
            xor_byte ^= sbus_frame[i];
        }
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