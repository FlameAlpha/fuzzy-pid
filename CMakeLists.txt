cmake_minimum_required(VERSION 3.17)
project(fuzzy-pid C)

set(CMAKE_C_STANDARD 99)

add_executable(fuzzy-pid fuzzyPID.c fuzzyPID.h example.c)