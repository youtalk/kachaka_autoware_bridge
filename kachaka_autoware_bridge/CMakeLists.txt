cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_bridge)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
