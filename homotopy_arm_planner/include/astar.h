/*
 * Copyright (c) 2018, Ramkumar Natarajan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * \file   astar.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.com)
 * \date   September 2018
 */

// homotopy_planner
#include <config.h>
#include <hash_functions.h>
#include <discrete_arm_planner.h>
#include <h_signature.h>

// STL
#include <unordered_map>
#include <memory>
#include <chrono>
#include <thread>
#include <math.h>
#include <vector>
#include <list>
#include <iostream>
#include <cstdio>

namespace homotopy_planner
{
  

typedef std::chrono::high_resolution_clock Clock;

class AStar : public DiscreteArmPlanner
{

public:

  AStar(double*	map,
        int x_size,
        int y_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        Point2D end_effector_goal=Point2D());

  void permuteWithRepetition(const char *str,
                             std::string prefix,
                             const int n,
                             const int r,
                             std::vector<std::string> &perm_sequence);

  std::vector<VertexPtr> getValidSuccessors(const VertexPtr& current_vertex);

  double euclideanDistance(const Point2D& point_a,
                           const Point2D& point_b);

  bool getCost(const VertexPtr& current_vertex,
               const VertexPtr &successor,
               double& cost);

  bool run(ArmState& start_state, ArmState& goal_state);

  VertexPtr stateToVertex(ArmState& state);

  std::vector<VertexPtr> getPath();

  std::vector<Point2D> getEndEffectorPath();

private:

  HSignaturePtr hsign_handle_;

  PriorityQueue open_;

  std::unordered_map<ArmState,
                     bool,
                     std::hash<ArmState>,
                     ArmStateEqual> closed_;

  std::unordered_map<ArmState,
                     VertexPtr,
                     std::hash<ArmState>,
                     ArmStateEqual> explored_;

  std::vector<VertexPtr> path_;

  std::vector<Point2D> end_effector_path_;

  // End effector planning
  Point2D end_effector_goal_;

  std::vector<std::vector<int>> suffix_list_;
};
  

} // namespace homotopy_planner

