cmake_minimum_required(VERSION 2.8.9)
project(pdr_hw_1)

#Bring the headers
include_directories(include
                    /usr/local/MATLAB/R2018a/extern/include)

#Can manually add the sources using the set command as follows:

#set(INCLUDES include/dijkstra_basic.h
#        include/hash_functions.h
#        include/heap.h
#        include/config.h)

set(INCLUDES include/dijkstra.h
             include/hash_functions.h
             include/heap.h
             include/config.h)
set(SOURCES planner.cpp src/dijkstra.cpp)

#Generate the shared library from the sources
add_library(hw_1 SHARED
            ${INCLUDES}
            ${SOURCES})
set_target_properties(hw_1 PROPERTIES COMPILE_FLAGS "-std=c++0x")
