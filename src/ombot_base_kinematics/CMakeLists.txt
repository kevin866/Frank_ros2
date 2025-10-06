cmake_minimum_required(VERSION 3.10)
project(ombot_base_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(mecanum_kinod src/mecanum_kinod.cpp)
target_include_directories(mecanum_kinod PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_kinod
  rclcpp geometry_msgs nav_msgs std_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS mecanum_kinod
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install headers so others (or you after install) can include them
install(DIRECTORY include/
  DESTINATION include)

# Export include dir for downstream packages (optional but good practice)
ament_export_include_directories(include)

# Keep if you want to install the manifest file (optional—ament normally handles this)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
