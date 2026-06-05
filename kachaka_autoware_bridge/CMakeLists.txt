cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/publish_canned_trajectory scripts/publish_initialization_state
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_trajectory_gen test/test_trajectory_gen.py)
endif()

ament_package()
