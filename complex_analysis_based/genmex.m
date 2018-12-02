setenv('LD_RUN_PATH', './cmake-build-debug')
% mex planner.cpp -Iinclude/ -Lcmake-build-debug/ -lhw_1
mex COMPFLAGS='-std=c++11' planner.cpp src/dijkstra_basic.cpp -Iinclude
% mex -g COMPFLAGS=' -DDEBUG -std=c++11' planner.cpp src/dijkstra_basic.cpp -Iinclude