cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_vehicle_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/control_to_twist_converter.cpp
  src/operation_mode_state_machine.cpp
  src/velocity_status_publisher.cpp
  src/vehicle_interface_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_control_to_twist_converter
    test/test_control_to_twist_converter.cpp
  )
  target_link_libraries(test_control_to_twist_converter ${PROJECT_NAME})

  ament_add_gtest(test_operation_mode_state_machine
    test/test_operation_mode_state_machine.cpp
  )
  target_link_libraries(test_operation_mode_state_machine ${PROJECT_NAME})

  ament_add_gtest(test_velocity_status_publisher
    test/test_velocity_status_publisher.cpp
  )
  target_link_libraries(test_velocity_status_publisher ${PROJECT_NAME})

  ament_add_gtest(test_vehicle_interface_node
    test/test_vehicle_interface_node.cpp
    TIMEOUT 30
  )
  target_link_libraries(test_vehicle_interface_node ${PROJECT_NAME})
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE launch config)
