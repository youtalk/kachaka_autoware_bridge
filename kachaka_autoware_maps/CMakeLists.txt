cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_maps)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/snapshot_map DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pcd_io test/test_pcd_io.py)
  ament_add_pytest_test(test_sidecars test/test_sidecars.py)
endif()

ament_package()
