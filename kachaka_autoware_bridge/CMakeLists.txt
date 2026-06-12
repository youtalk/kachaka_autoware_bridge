cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/localization_monitor
           scripts/perception_stub
           scripts/publish_canned_trajectory
           scripts/publish_initialization_state
           scripts/run_endurance
           scripts/set_loop_route
           scripts/tf_gate DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_trajectory_gen test/test_trajectory_gen.py)
  ament_add_pytest_test(test_loop_route test/test_loop_route.py)
  ament_add_pytest_test(test_endurance test/test_endurance.py)
  ament_add_pytest_test(test_se3 test/test_se3.py)
  ament_add_pytest_test(test_locmon test/test_locmon.py)
  ament_add_pytest_test(test_tf_gate test/test_tf_gate.py)
endif()

ament_package()
