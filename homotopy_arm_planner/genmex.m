% Debug mode
mex -g COMPFLAGS=' -DDEBUG -std=c++11' planner.cpp src/discrete_arm_planner.cpp src/astar.cpp -Iinclude

% % Release mode
% mex COMPFLAGS=' -std=c++11' planner.cpp src/discrete_arm_planner.cpp src/astar.cpp -Iinclude