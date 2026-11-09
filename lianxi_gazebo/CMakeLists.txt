cmake_minimum_required(VERSION 3.0.2)
project(lianxi_gazebo)

find_package(catkin REQUIRED COMPONENTS
gazebo_ros
robot_state_publisher
urdf
xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS gazebo_ros robot_state_publisher urdf xacro
#  DEPENDS system_lib
)

include_directories(
${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch urdf
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 新增这一段
install(PROGRAMS
  scripts/virtual_dynamic_obstacle.py
  scripts/nav_mon.py
  scripts/dete.py
  scripts/way_point_many
  scripts/laser_obstacle_tracker
  scripts/perfer_left_simple
  scripts/teb_auto_compare
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
