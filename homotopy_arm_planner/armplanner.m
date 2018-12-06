function[armplan] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
[armplan, armplanlength] = planner(envmap, armstart, armgoal, planner_id);

armplan = [armplan; armgoal];
% armplan
