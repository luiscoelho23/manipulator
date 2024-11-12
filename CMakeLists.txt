cmake_minimum_required(VERSION 3.5)
project(manipulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
#include_directories(include)

## Create Cpp executable
#add_executable(get_ang_xyz src/get_ang_xyz.cpp)
#ament_target_dependencies(get_ang_xyz rclcpp
#                                      mplibrary)

## Install Cpp executables
#install(TARGETS
#  get_ang_xyz
#  DESTINATION lib/${PROJECT_NAME}
#)

#target_link_libraries( get_ang_xyz
#    mplibrary::mplibrary
#)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  manipulator/controller.py
  manipulator/load_dmps.py
  manipulator/load_rl.py

  manipulator/rl/kdl_parser.py
  manipulator/rl/nn.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config resources
  DESTINATION share/${PROJECT_NAME})
  
  
ament_package()