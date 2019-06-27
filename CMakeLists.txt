cmake_minimum_required(VERSION 2.8.3)
project(quest)

find_package(catkin REQUIRED COMPONENTS
  roscpp mav_msgs sensor_msgs tf2_geometry_msgs fprime_msgs rospy rviz)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp sensor_msgs mav_msgs tf2_geometry_msgs fprime_msgs message_runtime
    DEPENDS
)
catkin_python_setup()

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES})
add_dependencies(teleop ${catkin_EXPORTED_TARGETS})

install(TARGETS teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/run_seq_client
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
