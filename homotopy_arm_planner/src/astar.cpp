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
 * \file   astar.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.com)
 * \date   September 2018
 */

// Project
#include <astar.h>

// STL
#include <algorithm>

namespace homotopy_planner
{

AStar::AStar(double*	map,
             int x_size,
             int y_size,
             double* armstart_anglesV_rad,
             double* armgoal_anglesV_rad,
             Point2D end_effector_goal) : end_effector_goal_(end_effector_goal),
                                          DiscreteArmPlanner(map,
                                                             x_size,
                                                             y_size,
                                                             armstart_anglesV_rad,
                                                             armgoal_anglesV_rad),
                                                             hsign_handle_(std::make_shared<HSignature>(this,
                                                                                                        map,
                                                                                                        x_size,
                                                                                                        y_size))
{
  end_effector_goal_ = getEndEffectorPose(armgoal_anglesV_rad_, NUM_DOF);
}

// The main recursive method to print all possible strings of length "length"
void AStar::permuteWithRepetition(const char *str,
                                   std::string prefix,
                                   const int n,
                                   const int r,
                                   std::vector<std::string> &perm_sequence)
{
  if (r == 1)
  {
    for (int j = 0; j < n; j++)
    {
      perm_sequence.push_back(prefix + str[j]);
    }
  }//Base case: r = 1, print the string "r" times + the remaining letter
  else
  {
    // One by one add all characters from "str" and recursively call for "r" equals to "r"-1
    for (int i = 0; i < n; i++)
    {
      // Next character of input added
      permuteWithRepetition(str, prefix + str[i], n, r - 1, perm_sequence);
      // "r" is decreased, because we have added a new character
    }
  }
}

std::vector<VertexPtr> AStar::getValidSuccessors(const VertexPtr& current_vertex)
{
  char sign[] = {'0', '1', '2'};
  int n = sizeof sign;

  std::vector<std::string> successor_sign;
  permuteWithRepetition(sign, "", n, NUM_DOF, successor_sign);

  std::vector<VertexPtr> successors;
  for (int i=0; i<successor_sign.size(); ++i)
  {
    if (successor_sign[i] == "000")
    {
      continue;
    }

    ArmState successor_state;

    for (int j=0; j<NUM_DOF; ++j)
    {
      switch (successor_sign[i][j])
      {
        case '0':
          successor_state.q_[j] = current_vertex->state_.q_[j];
          break;
        case '1':
          successor_state.q_[j] = normalize_angle_positive(current_vertex->state_.q_[j] +
                                                       static_cast<int>(DISCRETIZATION*DOUBLE_TO_INT_FACTOR));
          break;
        case '2':
          successor_state.q_[j] = normalize_angle_positive(current_vertex->state_.q_[j] -
                                                       static_cast<int>(DISCRETIZATION*DOUBLE_TO_INT_FACTOR));
          break;
      }
    }

#if COST_TYPE_HOMOTOPY_SPACE
    successor_state.signature_ = hsign_handle_->updateSignature(current_vertex->state_,
                                                                successor_state);
#endif

    bool current_state_itself = true;
    for (int j=0; j<NUM_DOF; ++j)
    {
      current_state_itself = current_state_itself &&
                             (successor_state.q_[j] == current_vertex->state_.q_[j]);
    }
    current_state_itself = current_state_itself && hsign_handle_->signatureEqual(successor_state.signature_,
                                                                                 current_vertex->state_.signature_);
    if (current_state_itself)
    {
      continue;
    }

    if (!IsValidArmConfiguration(successor_state.q_, NUM_DOF, map_, x_size_, y_size_))
    {
      continue;
    }

    // Suffix pruning
    bool valid_successor_sign = successor_sign.empty()? true: false;
    if (!successor_sign.empty())
    {
      for (int j=0; j<suffix_list_.size(); ++j)
      {
        if (hsign_handle_->signatureEqual(successor_state.signature_, suffix_list_[j]))
        {
          valid_successor_sign = true;
          break;
        }
      }
    }

    if (!valid_successor_sign)
    {
      continue;
    }

    if (explored_.find(successor_state) == explored_.end())
    {
      VertexPtr successor =
              std::make_shared<Vertex>(successor_state, std::numeric_limits<double>::infinity());
      successors.push_back(successor);
      explored_[successor_state] = successor;
    }
    else
    {
      successors.push_back(explored_[successor_state]);
    }
  }

  return successors;
}

double AStar::euclideanDistance(const Point2D &point_a,
                                const Point2D &point_b)
{
  return std::sqrt(std::pow(point_a.x_-point_b.x_,2) +
                   std::pow(point_a.y_-point_b.y_,2));
}

bool AStar::getCost(const VertexPtr &current_vertex,
                       const VertexPtr &successor,
                       double& cost)
{
#if COST_TYPE_JOINT_SPACE
  cost = current_vertex->g_cost_ +
         distanceBetweenVertices(current_vertex, successor);
  return true;
#elif COST_TYPE_TASK_SPACE
  DiscreteArmPlanner::Point2D current_end_effector_pose =
          getEndEffectorPose(current_vertex->state_.q_, NUM_DOF);
  DiscreteArmPlanner::Point2D successor_end_effector_pose =
          getEndEffectorPose(successor->state_.q_, NUM_DOF);

  cost = current_vertex->g_cost_ +
         euclideanDistance(current_end_effector_pose, successor_end_effector_pose);
  return true;
#elif COST_TYPE_HOMOTOPY_SPACE
  DiscreteArmPlanner::Point2D current_end_effector_pose =
          getEndEffectorPose(current_vertex->state_.q_, NUM_DOF);
  DiscreteArmPlanner::Point2D successor_end_effector_pose =
          getEndEffectorPose(successor->state_.q_, NUM_DOF);

  cost = current_vertex->g_cost_ +
         euclideanDistance(current_end_effector_pose, successor_end_effector_pose) +
         hsign_handle_->computeSignatureMismatch(current_vertex->state_.signature_,
                                                 successor->state_.signature_);
  return true;
#endif
}

bool AStar::run(ArmState& start_state,
                ArmState& goal_state)
{
  VertexPtr start_node;
  start_node = std::make_shared<Vertex>(start_state, 0.0);
  explored_[start_state] = start_node;
  open_.insert(start_node);
  open_.decreaseKey(start_node, 0.0);

  // Suffix list
  for (int i=0; i<goal_state.signature_.size(); ++i)
  {
    std::vector<int> suffix;
    for (int j=0; j<i+1; ++j)
    {
      suffix.push_back(goal_state.signature_[j]);
    }
    suffix_list_.push_back(suffix);
  }

  bool goal_found = false;

#if DEBUG
  int num_expansions = 0;

  // Best soln in time
  using namespace std::chrono;
  typedef std::chrono::high_resolution_clock Clock;
  auto start_time = Clock::now();
  double time_limit = 25.0;
  ArmState nearest_state = start_state;
  double nearest_state_dist = std::numeric_limits<double>::infinity();

  std::cout << "dist between start and goal: " << distanceBetweenVertices(start_state.q_, goal_state.q_) << std::endl;

#endif

  while(open_.size() > 0)
  {

    VertexPtr least_cost_vertex =
      std::static_pointer_cast<Vertex>(open_.remove());

    closed_[least_cost_vertex->state_] = true;

#if COST_TYPE_JOINT_SPACE
    if (closed_.find(goal_state) != closed_.end())
    {
      std::cout << "Goal node expanded. Terminating search!" << std::endl;
      goal_found = true;
      break;
    }
#elif COST_TYPE_TASK_SPACE
    DiscreteArmPlanner::Point2D least_cost_end_effector_pose =
            getEndEffectorPose(least_cost_vertex->state_.q_, NUM_DOF);

    if (euclideanDistance(least_cost_end_effector_pose, end_effector_goal_) < 0.1)
    {
      for (int i=0; i<NUM_DOF; ++i)
      {
        goal_state.q_[i] = least_cost_vertex->state_.q_[i];
      }
      std::cout << "Goal node expanded. Terminating search!" << std::endl;
      goal_found = true;
      break;
    }
#elif COST_TYPE_HOMOTOPY_SPACE
    if (least_cost_vertex->state_.signature_.size() == goal_state.signature_.size())
    {
      DiscreteArmPlanner::Point2D least_cost_end_effector_pose =
        getEndEffectorPose(least_cost_vertex->state_.q_, NUM_DOF);
    
      if (euclideanDistance(least_cost_end_effector_pose, end_effector_goal_) < 0.1)
      {
//        goal_found = true;
//        for (int j=0; j<goal_state.signature_.size(); ++j)
//        {
//          goal_found = goal_found && (least_cost_vertex->state_.signature_[j] == goal_state.signature_[j]);
//        }
        
//        if (goal_found)
//        {
          for (int i=0; i<NUM_DOF; ++i)
          {
            goal_state.q_[i] = least_cost_vertex->state_.q_[i];
          }
          goal_found = true;
          std::cout << "Goal node expanded. Terminating search!" << std::endl;
          break;
//        }
      }
    }
#endif


#if DEBUG
    for (int i=0; i<least_cost_vertex->state_.signature_.size(); ++i)
    {
      std::cout << least_cost_vertex->state_.signature_[i] << "\t";
    }
    std::cout << std::endl;

//      if (closed_[goal_state])
//      {
//        std::cout << "Goal node expanded. Terminating search!" << std::endl;
//        break;
//      }

    ++num_expansions;

    if (distanceBetweenVertices(goal_state, least_cost_vertex->state_) < nearest_state_dist)
    {
      nearest_state = least_cost_vertex->state_;
      nearest_state_dist = distanceBetweenVertices(goal_state, least_cost_vertex->state_);
    }

    auto end_time = Clock::now();
    if (duration_cast<duration<double> >(end_time - start_time).count() > time_limit)
    {
      std::cout << "Exiting because time exceeded" << std::endl;
      goal_state = nearest_state;
      goal_found = true;
      break;
    }

#endif

    std::vector<VertexPtr> least_cost_vertex_successors =
      getValidSuccessors(least_cost_vertex);

    for(const VertexPtr &successor : least_cost_vertex_successors)
    {
      if(closed_.find(successor->state_) == closed_.end())
      {
        double new_cost;
        if (!getCost(least_cost_vertex, successor, new_cost))
        {
          continue;
        }

        if (new_cost < successor->g_cost_)
        {
          successor->g_cost_ = new_cost;
          successor->parent_ = least_cost_vertex;
#if DEBUG
          successor->depth_ = least_cost_vertex->depth_+1;
#endif

          double f_cost;
#if COST_TYPE_JOINT_SPACE
          f_cost = new_cost +
                   HEURISTIC_INFLATION*distanceBetweenVertices(successor->state_, goal_state);
#elif COST_TYPE_TASK_SPACE
          DiscreteArmPlanner::Point2D successor_end_effector_pose =
                  getEndEffectorPose(successor->state_.q_, NUM_DOF);

          f_cost = new_cost +
                   HEURISTIC_INFLATION*std::sqrt(std::pow(successor_end_effector_pose.x_-end_effector_goal_.x_,2) +
                                                 std::pow(successor_end_effector_pose.y_-end_effector_goal_.y_,2));
#elif COST_TYPE_HOMOTOPY_SPACE
          DiscreteArmPlanner::Point2D successor_end_effector_pose =
            getEndEffectorPose(successor->state_.q_, NUM_DOF);
  
          f_cost = new_cost +
                   HEURISTIC_INFLATION*std::sqrt(std::pow(successor_end_effector_pose.x_-end_effector_goal_.x_,2) +
                                                 std::pow(successor_end_effector_pose.y_-end_effector_goal_.y_,2)) +
                   1000*hsign_handle_->computeSignatureMismatch(successor->state_.signature_,
                                                               goal_state.signature_);
#endif
          if (successor->heap_index_ == -1)
          {
            open_.insert(successor);
            open_.decreaseKey(successor,
                              f_cost);
          }
          else
          {
            open_.decreaseKey(successor,
                              f_cost);
          }
        }
      }
    }
  }

#if DEBUG
  std::cout << "Total number of expansions: " << num_expansions << std::endl;

  if (open_.size() == 0)
  {
    std::cout << "Open list empty" << std::endl;
    if (explored_.find(nearest_state) == explored_.end())
    {
      std::cout << "Nearest state not explored " << std::endl;
    }
    goal_state = nearest_state;
    goal_found = true;
  }
#endif

  if (goal_found)
  {
    path_.clear();

    path_.push_back(stateToVertex(goal_state));

    VertexPtr backtrack_vertex = stateToVertex(goal_state);
    while (backtrack_vertex->parent_)
    {
      backtrack_vertex = backtrack_vertex->parent_;
      path_.push_back(backtrack_vertex);
    }
    std::reverse(path_.begin(), path_.end());
  }
  else
  {
    path_.push_back(start_node);
    path_.push_back(stateToVertex(goal_state));
  }

  return true;
}

VertexPtr AStar::stateToVertex(ArmState &state)
{
  return explored_[state];
}

std::vector<VertexPtr> AStar::getPath()
{
  return path_;
}

std::vector<DiscreteArmPlanner::Point2D> AStar::getEndEffectorPath()
{
  end_effector_path_.clear();
  for (const auto& configuration : path_)
  {
    Point2D end_effector_pose = getEndEffectorPose(configuration->state_.q_,
                                                   NUM_DOF);
    end_effector_path_.push_back(end_effector_pose);
  }
  return end_effector_path_;
}

} // namespace homotopy_planner


