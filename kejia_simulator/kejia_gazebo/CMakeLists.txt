cmake_minimum_required(VERSION 2.8.3)
project(kejia_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS  "${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)



