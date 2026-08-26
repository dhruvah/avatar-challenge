cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/shape_tracer_node.py
  scripts/designer_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  web
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
