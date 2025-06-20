cmake_minimum_required(VERSION 3.8)
project(test_joint_wrench_msgs_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rclcpp
  rclcpp_action
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(test_joint_wrench_msgs_action_server_node src/test_joint_wrench_msgs_action_server_node.cpp)

target_include_directories(test_joint_wrench_msgs_action_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(test_joint_wrench_msgs_action_server_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_features(test_joint_wrench_msgs_action_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(test_joint_wrench_msgs_action_client_node src/test_joint_wrench_msgs_action_client_node.cpp)

target_include_directories(test_joint_wrench_msgs_action_client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(test_joint_wrench_msgs_action_client_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS test_joint_wrench_msgs_action_server_node test_joint_wrench_msgs_action_client_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
