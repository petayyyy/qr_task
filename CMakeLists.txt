cmake_minimum_required(VERSION 3.0.2)
project(qr_checker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

# Install Python node
install(PROGRAMS scripts/qr_checker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install web files
install(DIRECTORY www/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/www
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/qr_checker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)