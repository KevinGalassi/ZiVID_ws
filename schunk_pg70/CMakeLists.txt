cmake_minimum_required(VERSION 2.8.3)
project(schunk_pg70)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
  tf
  )

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)
                   
SET(SOURCES ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control.cpp
            ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control_lib.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/pg70_rs232_control.h)

add_message_files(FILES message_control_position.msg)

add_service_files(FILES 
      reference.srv
      set_position.srv
			set_pvac.srv
      get_error.srv
      get_position.srv
      get_current.srv
      acknowledge_error.srv
			move_time.srv
			move_time_loop.srv
      stop.srv)



generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  DEPENDS roscpp serial tf
  CATKIN_DEPENDS message_runtime
  )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} schunk_pg70_gencpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pg70_control_position src/pg70_control_position.cpp)
target_link_libraries(pg70_control_position ${catkin_LIBRARIES})
add_dependencies(pg70_control_position schunk_pg70_gencpp)
