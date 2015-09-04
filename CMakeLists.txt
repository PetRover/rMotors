cmake_minimum_required(VERSION 3.3)

add_library(rMotors STATIC motors.cpp rMotors.h)
include_directories("./../rPowerSystems")
include_directories("./../rProtocols")
target_link_libraries(rMotors rPowerSystems)
target_link_libraries(rMotors rProtocols)