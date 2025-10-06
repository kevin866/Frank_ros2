cmake_minimum_required(VERSION 3.16)
project(ombot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes OPTIONAL)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch OPTIONAL)
install(DIRECTORY resource/ DESTINATION share/${PROJECT_NAME}/resource OPTIONAL)

ament_package()
