cmake_minimum_required(VERSION 2.8.3)
project(nav_points)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
COMPONENTS
move_base_msgs
geometry_msgs
tf
actionlib
roscpp)

find_package(Qt5 COMPONENTS
    Core
    Widgets
    REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/deecamp_ui
  src
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_points
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/nav_points.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} src/nav_points.cpp)

add_executable(deecamp_ui 

               src/deecamp_ui/mainwindow.cpp
               src/deecamp_ui/main.cpp
               
               include/deecamp_ui/mainwindow.h
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

target_link_libraries(deecamp_ui
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets)
