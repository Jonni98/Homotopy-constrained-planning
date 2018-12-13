% Debug mode
% mex -g COMPFLAGS=' -DDEBUG'  planner.cpp src/discrete_arm_planner.cpp src/astar.cpp src/h_signature.cpp -Iinclude -I/opt/ros/kinetic/include/opencv-3.3.1-dev ...
%     -lopencv_core3 -lopencv_imgproc3 -lopencv_features2d3 -L/opt/ros/kinetic/lib/x86_64-linux-gnu
mex -g COMPFLAGS=' -DDEBUG'  planner.cpp src/discrete_arm_planner.cpp src/astar.cpp src/h_signature.cpp -Iinclude -I/opt/ros/kinetic/include/opencv-3.3.1-dev ...
    -lopencv_core -lopencv_imgproc -lopencv_features2d -L/usr/lib/x86_64-linux-gnu

% % Release mode
% mex COMPFLAGS=' -std=c++11' planner.cpp src/discrete_arm_planner.cpp src/astar.cpp -Iinclude
