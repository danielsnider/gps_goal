cmake_minimum_required(VERSION 2.8.3)

project(gps_goal)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
  std_srvs
  cv_bridge
)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs rospy std_srvs cv_bridge
)

catkin_install_python(
  PROGRAMS nodes/gps_goal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
