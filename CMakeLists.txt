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
find_package(fcl REQUIRED) 
find_package(PkgConfig REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)

# Find Assimp properly
find_package(assimp QUIET)
if(NOT assimp_FOUND)
  # Try to find with PkgConfig as a fallback
  pkg_check_modules(ASSIMP assimp)
  if(NOT ASSIMP_FOUND)
    message(FATAL_ERROR "Could not find Assimp. Please install libassimp-dev.")
  endif()
endif()

pkg_check_modules(LIBCCD REQUIRED ccd)

# Include Cpp "include" directory
include_directories(
      include
      ${libstatistics_collector_INCLUDE_DIRS}
      ${LIBCCD_INCLUDE_DIRS}
      ${ASSIMP_INCLUDE_DIRS}
)

## Create Cpp executable
add_executable(run_cd  
      src/run_cd.cpp
      src/collision_detection_env.cpp
)

target_link_libraries(run_cd
      ${rclcpp_LIBRARIES}
      ${FCL_LIBRARIES}
      ${LIBCCD_LIBRARIES}
      ${urdf_LIBRARIES}
      ${ASSIMP_LIBRARIES}
      assimp
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts and modules
install(DIRECTORY
  manipulator/
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY launch resources config
  DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS run_cd 
  DESTINATION lib/${PROJECT_NAME}
)

find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_package()