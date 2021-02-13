cmake_minimum_required(VERSION 2.8.3)
project(zivid_msgs)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
   actionlib_msgs
   message_generation
   geometry_msgs
)

add_action_files(FILES
	moveWire.action
)

add_service_files( FILES
  newObjectName.srv
)


generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_generation
    message_runtime
)

