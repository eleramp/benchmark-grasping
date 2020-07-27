###########
## Build ##
###########

catkin_install_python(PROGRAMS
  grasp_planner_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

