cmake_minimum_required(VERSION 3.0.2)
project(vbats)

add_compile_options(-std=c++17)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ap_planning
  moveit_visual_tools
  traj_serv_to_action
)

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
#  INCLUDE_DIRS include
#  LIBRARIES ap_planning_examples
#  CATKIN_DEPENDS ap_planning
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(screw_planning_sim src/screw_planning_sim.cpp)
target_link_libraries(screw_planning_sim ${catkin_LIBRARIES})
add_dependencies(screw_planning_sim ${catkin_EXPORTED_TARGETS})

add_executable(vb_screw_planning src/vb_screw_planning.cpp)
target_link_libraries(vb_screw_planning ${catkin_LIBRARIES})
add_dependencies(vb_screw_planning ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(
  TARGETS screw_planning_sim vb_screw_planning
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
