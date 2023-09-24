cmake_minimum_required(VERSION 3.8)
project(adisha_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(adisha_interface REQUIRED)

# add include directories
include_directories(
  include 
  ${rclcpp_INCLUDE_DIRS}
)

# add custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointVal.msg"
  "msg/JointTorque.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# create executables and linking
add_executable(adisha_pose_editor
  src/adisha_pose_editor_main.cpp
  src/adisha_interface/adisha_pose_editor.cpp
)
ament_target_dependencies(adisha_pose_editor
  rclcpp
  std_msgs
  adisha_interface
)
target_link_libraries(adisha_pose_editor
  yaml-cpp
)

# install
install(TARGETS 
  adisha_pose_editor
  DESTINATION lib/${PROJECT_NAME}
)

# testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()