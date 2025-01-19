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

现在的问题：多线程没搞明白