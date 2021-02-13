cmake_minimum_required(VERSION 2.8.3)
project(zivid_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES statrion_ur5_e_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
