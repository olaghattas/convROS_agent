cmake_minimum_required(VERSION 3.8)
project(convros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
    action/ConversationRequest.action
)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    DEPENDENCIES
    action_msgs
    std_msgs
    builtin_interfaces
)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Install directories
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY action/
  DESTINATION share/${PROJECT_NAME}/action
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
