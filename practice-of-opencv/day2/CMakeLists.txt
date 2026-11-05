#1.声明所需的CMake最低版本
cmake_minimum_required(VERSION 3.5)

#2.给项目起个名字
project(Day2)

#3.设置C++标准,建议使用C++11 C++14
set(CMAKE_CXX_STANDARD 14)

#4.寻找系统中安装的opencv库
# REQUIRED的意思是:如果找不到就报错停止不能强行编译。
find_package(OpenCV REQUIRED)

#5.将OpenCV的头文件4路径包含进来(解决include找不到文件的问题)
include_directories(${OpenCV_INCLUDE_DIRS})

#6.指定要编译的源文件，并生成可执行文件，语法是：add_executable(生成的可执行文件名 源文件名)
add_executable(main main.cpp)

#7.将生成的可执行文件与opencv库连接起来（解决unfined reference 报错）
target_link_libraries(main ${OpenCV_LIBS})