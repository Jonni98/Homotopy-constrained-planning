cmake_minimum_required(VERSION 2.8.9)
project(pdr_hw_1)

#Bring the headers
include_directories(include
                    /usr/local/MATLAB/R2018a/extern/include)

#Can manually add the sources using the set command as follows:

set(INCLUDES include/discrete_arm_planner.h
        include/astar.h
             include/hash_functions.h
             include/heap.h
             include/config.h
             include/macros.h)
set(SOURCES planner.cpp src/astar.cpp src/discrete_arm_planner.cpp)

#Generate the shared library from the sources
add_library(hw_1 SHARED
            ${INCLUDES}
            ${SOURCES})
set_target_properties(hw_1 PROPERTIES COMPILE_FLAGS "-std=c++0x")
