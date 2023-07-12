cmake_minimum_required(VERSION 3.5)
project(cslam_storage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # ${RTABMap_INCLUDE_DIRS}
  # ${rtabmap_ros_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  # ${GTSAM_INCLUDE_DIR}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cslam_common_interfaces REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY 
   config
   DESTINATION share/${PROJECT_NAME}/
)

# Install Python executables
install(PROGRAMS
  cslam_storage/cslam_storage.py
  cslam_storage/map_recovery_node.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
