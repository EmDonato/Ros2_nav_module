cmake_minimum_required(VERSION 3.8)
project(navigation_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/velocity_mux.py DESTINATION lib/${PROJECT_NAME} RENAME velocity_mux)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  FILES ../../config/navigation.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_velocity_mux test/test_velocity_mux.py)
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
  ament_add_pytest_test(test_navigation_smoke test/test_navigation_smoke.py TIMEOUT 180)
endif()

ament_package()
