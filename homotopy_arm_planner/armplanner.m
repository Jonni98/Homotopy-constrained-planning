function[armplan,endeffector_armplan] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
[armplan, endeffector_armplan, armplanlength] = planner(envmap, armstart, armgoal, planner_id);

armplan = [armplan; armgoal];
% armplan
