cmake_minimum_required(VERSION 2.8.12)
project(image_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        nav_msgs
        random_numbers
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        message_filters
        message_generation
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

##################
## ROS messages ##
##################
add_message_files(
        FILES
        FeatureMeasurement.msg
        CameraMeasurement.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)
# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_node
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs random_numbers roscpp rospy sensor_msgs std_msgs tf message_runtime
  DEPENDS  Boost EIGEN3 OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_node 
               src/image_node.cpp)
target_link_libraries(image_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_dependencies(image_node ${catkin_EXPORTED_TARGETS})
