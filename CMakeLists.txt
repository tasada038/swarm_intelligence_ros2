cmake_minimum_required(VERSION 3.5)
project(swarm_intelligence_ros2)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Create Cpp executable
add_executable(pso_topic
  src/pso/pso_topic/pso_topic_node.cpp
  src/pso/pso_topic/pso_topic_component.cpp
)
ament_target_dependencies(pso_topic
  rclcpp std_msgs
)

add_executable(mrfo_node src/mrfo/mrfo_node.cpp src/mrfo/mrfo_component.cpp)
ament_target_dependencies(mrfo_node
  rclcpp std_msgs
)

# Install Cpp executables
install(TARGETS
  pso_topic
  mrfo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()