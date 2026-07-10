cmake_minimum_required(VERSION 3.5)
project(kachaka_autoware_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_mount test/test_urdf_mount.py)
endif()

ament_package()
