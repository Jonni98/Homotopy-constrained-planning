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
 * \file   dijkstra_basic.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.com)
 * \date   September 2018
 */

// homotopy_planner
#include <config.h>
#include <hash_functions.h>

// STL
#include <unordered_map>
#include <memory>
#include <chrono>
#include <thread>
#include <math.h>
#include <vector>
#include <list>
#include <chrono>
#include <iostream>
#include <cstdio>

namespace homotopy_planner
{
  
  class Environment;
  struct Vertex2D;
  class ForwardSearch;
  
  typedef std::chrono::high_resolution_clock Clock;
  typedef std::shared_ptr<Environment> EnvironmentPtr;
  typedef std::shared_ptr<Vertex2D> Vertex2DPtr;
  typedef std::shared_ptr<ForwardSearch> ForwardSearchPtr;
  
  enum Params
  {
    TRANSITION_COST = 0,
    TIME_LEWAY_FACTOR = 3
  };
  
  struct Vertex2D : public PriorityQueue::Element
  {
    Vertex2D(State2DPtr& state);
  
    State2DPtr state_;
    
    Vertex2DPtr parent_vertex_;
    
    int root_id_;
    
    int depth_;
  };
  
  class Environment
  {
  public:
    
    Environment(double*	map,
                int collision_thresh,
                int x_size,
                int y_size);
    
    ~Environment();
    
    double getCollisionThreshold() const;
    
    bool getCost(const int x,
                 const int y,
                 double& cost) const;
    
    double* getHeuristicMap() const;
  
    double* getForwardSearchMap() const;
  
    double getHeuristicCost(const int x,
                            const int y) const;
    
    bool setHeuristicCost(const int x,
                          const int y,
                          double cost);
  
    bool setForwardSearchCost(const int x,
                              const int y,
                              double cost);
  
    // Used only during the forward search and assumes that
    // the backward search is already finished
    PriorityQueue total_cost_;

//  private:
    
    double* map_;
    int collision_thresh_;
    int x_size_;
    int y_size_;
    
    double* heuristic_map_;
    
    double* forward_search_map_;
    
    std::vector<double> target_traj_cummulative_cost_;
    
    Clock::time_point start_time_;
  };
  
  class Dijkstra
  {
  
  public:
    
    Dijkstra(EnvironmentPtr& environment_ptr);
    
    std::vector<Vertex2DPtr> getValidSuccessors(const Vertex2DPtr& current_vertex);
    
    bool getCost(const Vertex2DPtr& current_vertex,
                 const Vertex2DPtr &successor,
                 double& cost);
    
    bool run(std::vector<State2DPtr>& start_states,
             bool search_forward);

    Vertex2DPtr stateToVertex(State2DPtr& state);

//  private:
  
    EnvironmentPtr environment_ptr_;
    
    PriorityQueue open_;
    
    std::unordered_map<State2DPtr,
                       bool,
                       std::hash<State2DPtr>,
                       State2DPtrEqual> closed_;
    
    std::unordered_map<State2DPtr,
                       Vertex2DPtr,
                       std::hash<State2DPtr>,
                       State2DPtrEqual> explored_;
    
  };
  

  class ForwardSearch
  {
  public:
  
    ForwardSearch(int robot_pose_x,
                  int robot_pose_y,
                  EnvironmentPtr& environment_ptr);
    
    bool run();

//  private:
    
    int robot_pose_x_;
    int robot_pose_y_;
  
    Dijkstra planner_;
  };
  
} // namespace homotopy_planner

