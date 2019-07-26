cmake_minimum_required(VERSION 2.8.3)
project(madrob)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()


install(PROGRAMS
  nodes/bridge_handle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
