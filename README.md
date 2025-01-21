

# ROS Humble

需要预先下载serial库为ros使用

```sh
sudo apt-get install ros-humble-serial-driver
```



# UartToSbus

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

使用SSCOM循环发送数据，确认可以收到

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

