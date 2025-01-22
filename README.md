

# UartToSbus项目

​	适配Uart转sbus模块的cpp与ROS代码，包含测试代码与实机部署代码，较为混乱，生命周期在开发完成后就将结束。由于PX4 OFFBOARD模式在没有遥控器的情况下进行开发会存在一些不便，所以采用本模块予以弥补。

以下记录基本信息、使用方法与开发过程。（2025.1.22）

# 项目使用方法

## 概况

​	文件夹内外围各种源码（包含Qt在内）为开发测试用代码（生命周期已结束，存在未修改的bug），而ROS_humble代码生命周期也结束了，仅ROS_galactic是最终部署到Jetson上的版本，包含chassis_controller功能包（含有k跨平台serial库代码@Copyright William Woodall and John Harrison，即开即用）和带有ui的chassis_controller_channels功能包。后续合并到主分支的代码将修改话题消息格式，请注意：

```sh
vx%fvy%fwz%f
# 如vx0.1vy-0.8wz0
```

## 原理

本模块可将35位uart信号帧转换为25位sbus信号帧，具体35帧格式为：

```cpp
#define SBUS_FRAME_HEAD 0x0F //帧头固定与sbus相同
#define SBUS_FLAGS 0x00 //一定注意这里 PX4对flag的定义与商店官方解释不同

static uint8_t sbus_frame[35] = {0x00}; //uart转sbus前的帧，35位长
static uint16_t channels[16]; // 32 Bytes for 16 channels, Big endian

// define frame head
sbus_frame[0] = SBUS_FRAME_HEAD;

// define channels from byte 1 to byte 31
for (int i = 0; i < 16; i++)
    sbus_frame[2 * i + 1] = channels[i] >> 8,
                       sbus_frame[2 * i + 2] = channels[i] & 0xff;

// define frame tail
sbus_frame[33] = SBUS_FLAGS;
sbus_frame[34] = sbus_xor();


uint8_t sbus_xor()
{
    uint8_t xor_byte = 0;
    for (int i = 1; i <= 33; i++)
        xor_byte ^= sbus_frame[i];

    return xor_byte;
}
```



### 注意以下三点重要内容：

1. Uart channel值为0～2000，中位为1000，对应PX4 sbus值876～2124，中位值1499。线性对应关系；

2. 发送频率自动调整，程序发送20Hz经过测试无问题；

3. uart33 flag位根据px4源码设为0x00无问题（sbus32位 #define SBUS_FAILSAFE_BIT   3

   \#define SBUS_FRAMELOST_BIT  2  ）；uart34 校验位为1~33位的异或值。

# ROS_galatic代码

进入ROS_galatic工作空间后开始编译，完成后启动chassis_controller chassis_control节点，节点具有以下功能：

* 检查Uart连接并尝试失败重连；
* 接收话题消息并转化成uart帧发送；
* 看门狗：0.5s内无话题传输则停止各通道输出，5s输出一警告表示状态

如需测试，可启动其他两个节点中的一个（注意话题消息区别）

# 开发日志

最终总结一下开发过程中的问题吧

通过这次开发，我们发现基于DDS的ROS在程序编写与数据处理方面真的是简单得不得了。通过在windows平台完成模块测试，在linux平台完成多线程程序与ROS程序编写，我们积累了一些经验。简单总结下其中比较重要的：

## serial独立库代码的引入

下载serial库后始终不能引入，故在ROS_humble开发过程中选择保留serial代码，由于需要编译，故需要向CmakeList.txt中添加：

```cmake
file(GLOB SERIAL_SOURCES "src/serial/*")

include_directories(
  include
  src/serial
)

add_executable(chassis_control src/chassis_controller.cpp ${SERIAL_SOURCES})
```

无需向package.xml中特别添加其他东西，colcon即可找到这个不包含节点代码也不在环境变量中的库代码，根据引用关系予以编译。

## 串口发送与异常捕获

### 异常检查

```cpp
        try
        {
            SbusPort.write(sbus_frame, 35); // Sends data to the downloader via serial port
        }
        catch (serial::IOException &e)
        {
            std::cerr << "Unable to send data through serial port: " << e.what() << std::endl;
        }
```

### 异常阻塞重连

```cpp
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
```

其实这里通过检查USB是否存在而检查是否连接，并不能确保模块与通信链路的完整。在Jetson上则有检查THS0权限问题的作用。

# Depricated Context

用于给机器人使用，在没有全场定位的情况下解锁px4飞控并控制，以及以后做一些有意思的东西使用。

无耻抄袭了William Woodall and John Harrison的代码，因为本人实在太弱，没有写出跨平台代码的功力，无比感谢。。。求放过

<img src="README.assets/image-20241123011044708.png" alt="image-20241123011044708" style="zoom:25%;" />



Ubuntu下编译命令：

```shell
g++ -g -std=c++11 -I../serial ../chassis_controller.cpp ../sbus.cpp ../test.c ../serial/serial.cc ../serial/unix.cc -o test
```

想要执行，必须

```shell
./test
```

## 开发日志

从10月29日开始开发，中途一度废弃

编译命令：在build下

```shell
g++ -g -std=c++11 -I..\serial ..\chassis_controller.cpp ..\sbus.cpp ..\test.c ..\serial\serial.cc ..\serial\win.cc -o test
```

使用SSCOM循环发送数据，确认可以收到(1月22日批注：flag位都不对)

ch1 up == lat 2000

```
0F 07 08 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C E8
```

ch1 down==unlat 1000

```
0F 00 C8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C 2F
```

ch2 up==forward 2000

```
0F 03 E8 07 08 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C E8
```

ch2 down==backward 1000

```
0F 03 E8 00 C8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C 2F
```

ch3 up == max

```
0F 03 E8 03 E8 07 08 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C E8
```

ch3 down == idle

```
0F 03 E8 03 E8 00 C8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C 2F
```

ch4 up == rotate 2000

```
0F 03 E8 03 E8 03 E8 07 08 05 DC 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C DA
```

ch4 down == unrot 1000

```
0F 03 E8 03 E8 03 E8 00 C8 05 DC 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C 1D
```

all mid == stop

```sh
0F 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 03 E8 0C 0C
```

