cmake_minimum_required(VERSION 3.10)

# 项目名称
project(Theseus_Mecanum_Sbus_test)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 设置编译器
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

# 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/library/serial)

# 添加可执行文件
add_executable(test
    test.c
    chassis_controller.cpp
    sbus.cpp
)

# 如果没有库需要链接，可以删除或注释掉以下行
# target_link_libraries(test <library_name>)