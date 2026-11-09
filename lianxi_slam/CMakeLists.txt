cmake_minimum_required(VERSION 3.0.2)
project(lianxi_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
gmapping
map_server
roscpp
rospy
tf
geometry_msgs
nav_msgs
sensor_msgs
gazebo_msgs
costmap_converter
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS gmapping map_server roscpp rospy tf geometry_msgs nav_msgs sensor_msgs gazebo_msgs costmap_converter
)

###########
## Build ##
###########
include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable(dwa_sim src1/dwa_sim.cpp)
add_executable(obs_auto src1/obs_auto.cpp)
add_executable(obs2_stop_and_decide_final src1/obs2_stop_and_decide_final.cpp)
add_executable(zidingyi_start src1/zidingyi_start.cpp)
add_executable(path_show src1/path_show.cpp)

target_link_libraries(dwa_sim
${catkin_LIBRARIES}
)
target_link_libraries(obs_auto
${catkin_LIBRARIES}
)
target_link_libraries(obs2_stop_and_decide_final
${catkin_LIBRARIES}
)
target_link_libraries(zidingyi_start
${catkin_LIBRARIES}
)
target_link_libraries(path_show
${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
