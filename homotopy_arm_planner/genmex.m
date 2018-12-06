
% mex planner.cpp -Iinclude/ -Lcmake-build-debug/ -lhw_1
% mex COMPFLAGS='-std=c++11' planner.cpp src/randomized_planner.cpp src/PRM.cpp src/RRT.cpp -Iinclude
mex -g COMPFLAGS=' -DDEBUG -std=c++11' planner.cpp src/discrete_arm_planner.cpp src/dijkstra.cpp -Iinclude