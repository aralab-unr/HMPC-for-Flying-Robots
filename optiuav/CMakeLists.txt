cmake_minimum_required(VERSION 2.8.3)
project(optiuav)
set(CMAKE_CXX_STANDARD 17)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(gazebo REQUIRED)
find_package(casadi REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  message_generation
  dynamic_reconfigure
  gazebo_plugins                    
  gazebo_ros 
  genmsg
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
add_executable(mpccontroller src/mpccontroller.cpp)
target_link_libraries(mpccontroller ${catkin_LIBRARIES} casadi)
add_executable(pathplanning src/pathplanning.cpp)
target_link_libraries(pathplanning ${catkin_LIBRARIES} casadi)
include_directories(${GAZEBO_INCLUDE_DIRS})            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(optiuavplugin SHARED src/optiuavplugin.cc)
target_link_libraries(optiuavplugin ${GAZEBO_LIBRARIES})

