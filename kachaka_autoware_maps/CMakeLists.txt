cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_maps)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
