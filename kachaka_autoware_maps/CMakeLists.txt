cmake_minimum_required(VERSION 3.14)
project(kachaka_autoware_maps)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/snapshot_map)
  install(PROGRAMS scripts/snapshot_map DESTINATION lib/${PROJECT_NAME})
endif()

install(
  DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pcd_io.py)
    ament_add_pytest_test(test_pcd_io test/test_pcd_io.py)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sidecars.py)
    ament_add_pytest_test(test_sidecars test/test_sidecars.py)
  endif()
endif()

ament_package()
