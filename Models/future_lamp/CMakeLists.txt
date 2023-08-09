cmake_minimum_required(VERSION 2.8.3)
project(vr_lamp)

find_package(catkin REQUIRED)

catkin_package()

set(DEST_PATH $ENV{HOME}/.gazebo/models/${PROJECT_NAME})

install(
  FILES model.sdf model.config
  DESTINATION ${DEST_PATH}
)

install(
  DIRECTORY meshes
  DESTINATION ${DEST_PATH}
)

