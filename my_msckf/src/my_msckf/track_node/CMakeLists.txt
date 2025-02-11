cmake_minimum_required(VERSION 2.8.3)
project(track_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# Modify cmake module path if new .cmake files are required
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        nav_msgs
        random_numbers
        roscpp
        rospy
        eigen_conversions
        tf_conversions
        sensor_msgs
        std_msgs
        tf
        message_filters
        message_generation
)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SuiteSparse REQUIRED)

##################
## ROS messages ##
##################

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
        LIBRARIES track_node
        CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs random_numbers roscpp
        eigen_conversions tf_conversions rospy sensor_msgs std_msgs tf message_filters message_runtime
        DEPENDS Boost EIGEN3 OpenCV SUITESPARSE
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(track_node 
               src/track_node.cpp)

target_link_libraries(track_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES})
add_dependencies(track_node ${catkin_EXPORTED_TARGETS})
