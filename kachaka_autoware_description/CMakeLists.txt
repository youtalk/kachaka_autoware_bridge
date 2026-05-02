cmake_minimum_required(VERSION 3.5)
project(kachaka_autoware_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
