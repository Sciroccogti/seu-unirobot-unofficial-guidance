cmake_minimum_required(VERSION 3.13)
project(ball C)

set(CMAKE_C_STANDARD 11)

include_directories(include)

link_directories(.)

add_executable(ball main.c)

target_link_libraries(ball libdarknet.so)