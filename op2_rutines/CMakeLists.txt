cmake_minimum_required(VERSION 3.8)
project(op2_rutines)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(op2_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_library(op2_rutines SHARED src/op2_rutines.cpp)

target_include_directories(op2_rutines PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(op2_rutines
  PRIVATE "OP2_RUTINES_CPP_BUILDING_DLL")
ament_target_dependencies(op2_rutines
  "op2_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface")
rclcpp_components_register_node(op2_rutines 
  PLUGIN "op2_rutines::Op2Task" 
  EXECUTABLE op2_rutines_node
)

install(TARGETS 
  op2_rutines
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
