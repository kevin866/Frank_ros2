cmake_minimum_required(VERSION 3.16)
project(ombot_bringup)

find_package(ament_cmake REQUIRED)

# Pure resource package: launch files + controller YAML
install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/     DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource OPTIONAL)

ament_package()
