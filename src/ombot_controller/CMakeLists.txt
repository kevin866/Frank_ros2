cmake_minimum_required(VERSION 3.16)
project(ombot_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)        

add_executable(mecanum_forward_cmd src/mecanum_forward_cmd.cpp)
ament_target_dependencies(mecanum_forward_cmd rclcpp geometry_msgs)
install(TARGETS mecanum_forward_cmd
  DESTINATION lib/${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED
  src/effort_controller.cpp
  src/gravity_cancel_controller.cpp
  src/joint_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rcpputils controller_interface realtime_tools
  trajectory_msgs rclcpp rclcpp_lifecycle std_msgs kdl_parser
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)



add_executable(ee_to_base src/ee_to_base.cpp)
ament_target_dependencies(ee_to_base
  rclcpp
  geometry_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)
install(TARGETS ee_to_base
DESTINATION lib/${PROJECT_NAME})          # <-- executables go here in ROS 2


# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ombot_controller.xml)

# Tools (optional)
# add_executable(send_trajectory src/reference_generator/send_trajectory.cpp)
# add_executable(send_trajectory_om src/reference_generator/send_trajectory_om.cpp)
# ament_target_dependencies(send_trajectory rclcpp trajectory_msgs)
# ament_target_dependencies(send_trajectory_om rclcpp trajectory_msgs)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES ombot_controller.xml DESTINATION share/${PROJECT_NAME})
# install(TARGETS send_trajectory send_trajectory_om
#   RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(pluginlib controller_interface realtime_tools trajectory_msgs rclcpp rclcpp_lifecycle std_msgs kdl_parser Eigen3)
ament_package()
