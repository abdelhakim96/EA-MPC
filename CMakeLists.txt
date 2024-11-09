cmake_minimum_required(VERSION 2.8.3)

# Set policy CMP0048 to NEW to allow VERSION to be specified
cmake_policy(SET CMP0048 NEW)

# Set the project version
project(bluerov2_nmpc VERSION 1.0.0)

set(PROJECT_NAME bluerov2_nmpc)
set(ROS_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ${CMAKE_CXX_FLAGS_DEBUG} -O1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

# Add the path to the submodule
#set(SNAPSTACK_MSGS_PATH ${PROJECT_SOURCE_DIR}/msg/snapstack_msgs)

# Find the snapstack_msgs package
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslib nav_msgs decomp_ros_msgs tf 	snapstack_msgs)

# Include the snapstack_msgs directory
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SNAPSTACK_MSGS_PATH}/cmake)
#find_package(snapstack_msgs REQUIRED)

find_package(Eigen3 3.0.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#############
# LIBRARIES #
#############

add_library(${PROJECT_NAME}_LIBRARY
#src/nmpc_pc.cpp
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/nmpc_qpoases_interface.cpp
  solver/nmpc_integrator.c
  solver/nmpc_solver.c
  solver/nmpc_auxiliary_functions.c
)

include_directories(${PROJECT_NAME}_LIBRARY PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/solver
  ${PROJECT_SOURCE_DIR}/solver/qpoases
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE
  ${PROJECT_SOURCE_DIR}/solver/qpoases/INCLUDE/EXTRAS
  ${PROJECT_SOURCE_DIR}/solver/qpoases/SRC
)

add_message_files(
  FILES
  PwmCmd.msg
  # Add the State message from snapstack_msgs
  #msg/State.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  #snapstack_msgs  # Ensure snapstack_msgs is a dependency
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

############
# BINARIES #
############

add_executable(bluerov2_nmpc src/nmpc_bluerov2_2D_main.cpp src/nmpc_bluerov2_2D.cpp)
target_link_libraries(bluerov2_nmpc ${PROJECT_NAME}_LIBRARY ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/thrust_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
