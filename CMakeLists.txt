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
find_package(cpp_srvcli_srv REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Create Cpp executable
add_executable(test_pso src/test/test_pso.cpp)
add_executable(test_mrfo src/test/test_mrfo.cpp)
add_executable(test_pso_ver2 src/test/test_pso_ver2.cpp)
add_executable(test_mrfo_ver2 src/test/test_mrfo_ver2.cpp)
add_executable(test_data src/test/float32pub_cpp_node.cpp)
add_executable(pso_topic 
  src/pso/pso_topic/pso_topic_node.cpp
  src/pso/pso_topic/pso_topic_component.cpp
)
add_executable(pso_service
  src/pso/pso_service/pso_service_node.cpp
  src/pso/pso_service/pso_service_component.cpp 
  src/pso/pso_service/particle_service.cpp
)
add_executable(mrfo_node
  src/mrfo/mrfo_node.cpp
  src/mrfo/mrfo_component.cpp 
)
ament_target_dependencies(test_data
  rclcpp std_msgs
)
ament_target_dependencies(pso_topic
  rclcpp std_msgs
)
ament_target_dependencies(pso_service
  rclcpp std_msgs cpp_srvcli_srv
)
ament_target_dependencies(mrfo_node
  rclcpp std_msgs
)
add_executable(server_test src/service/add_server_test.cpp)
ament_target_dependencies(server_test
  rclcpp std_msgs cpp_srvcli_srv)

add_executable(client_test src/service/view_client_test.cpp)
ament_target_dependencies(client_test
  rclcpp std_msgs cpp_srvcli_srv)

# Install Cpp executables
install(TARGETS
  test_pso
  test_mrfo
  test_pso_ver2
  test_mrfo_ver2
  server_test
  client_test
  test_data
  pso_topic
  pso_service
  mrfo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()