cmake_minimum_required(VERSION 2.8.9)
project(homotopy_planner)

find_package( OpenCV REQUIRED )

#Bring the headers
include_directories(include
                    /usr/local/MATLAB/R2018a/extern/include
                   /opt/ros/kinetic/include/opencv-3.3.1-dev)

#Can manually add the sources using the set command as follows:

set(INCLUDES include/discrete_arm_planner.h
             include/h_signature.h
             include/astar.h
             include/hash_functions.h
             include/heap.h
             include/config.h
             include/macros.h)
set(SOURCES planner.cpp src/astar.cpp src/discrete_arm_planner.cpp src/h_signature.cpp)

#Generate the shared library from the sources
add_library(homotopy_arm_planner SHARED
            ${INCLUDES}
            ${SOURCES})
set_target_properties(homotopy_arm_planner PROPERTIES COMPILE_FLAGS "-std=c++0x")
