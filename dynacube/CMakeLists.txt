cmake_minimum_required(VERSION 2.8.3)
project(dynacube)
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
add_executable(obmovementcube src/obmovementcube.cpp)
target_link_libraries(obmovementcube ${catkin_LIBRARIES})
add_executable(dynacubecontroller src/dynacubecontroller.cpp)
target_link_libraries(dynacubecontroller ${catkin_LIBRARIES} casadi)
add_executable(dynacubepath src/dynacubepath.cpp)
target_link_libraries(dynacubepath ${catkin_LIBRARIES} casadi)
include_directories(${GAZEBO_INCLUDE_DIRS})            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_library(dynacubeplugin SHARED src/dynacubeplugin.cc)
target_link_libraries(dynacubeplugin ${GAZEBO_LIBRARIES})
