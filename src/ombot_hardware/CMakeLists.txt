cmake_minimum_required(VERSION 3.16)
project(ombot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(dynamixel_sdk REQUIRED)         # if you use it
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ombot_arm_hardware.cpp
  src/ombot_base_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle pluginlib hardware_interface
  dynamixel_sdk
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Export the hardware plugin
pluginlib_export_plugin_description_file(hardware_interface ombot_hardware.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES ombot_hardware.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_export_dependencies(rclcpp rclcpp_lifecycle pluginlib hardware_interface dynamixel_sdk Eigen3)
ament_package()
