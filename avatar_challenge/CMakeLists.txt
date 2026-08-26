cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/shape_tracer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # These are plain pytest modules over pure-numpy code -- no ROS graph needed,
  # so they run under `colcon test` and standalone with pytest alike.
  ament_add_pytest_test(geometry_tests test/test_geometry.py)
  ament_add_pytest_test(kinematics_tests test/test_kinematics.py)
  ament_add_pytest_test(execution_safety_tests test/test_execution_safety.py)
endif()

ament_package()
