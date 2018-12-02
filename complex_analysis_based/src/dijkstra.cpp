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
 * \file   dijkstra_basic.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.com)
 * \date   September 2018
 */

#include <dijkstra.h>

namespace homotopy_planner
{
  
  Vertex2D::Vertex2D(State2DPtr &state) :
    state_(state)
  {
    key_ = std::numeric_limits<double>::infinity();
    parent_vertex_ = nullptr;
  }
  
  Environment::Environment(double *map,
                           int collision_thresh,
                           int x_size,
                           int y_size) : start_time_(Clock::now()),
                                         map_(map),
                                         collision_thresh_(collision_thresh),
                                         x_size_(x_size),
                                         y_size_(y_size),
                                         heuristic_map_(new double[x_size_ * y_size_]),
                                         forward_search_map_(new double[x_size_ * y_size_])
  {
    std::copy(map_,
              map_ + x_size_*y_size_,
              heuristic_map_);
  
    std::copy(map_,
              map_ + x_size_*y_size_,
              forward_search_map_);
  }
  
  Environment::~Environment()
  {
    delete[] heuristic_map_;
    delete[] forward_search_map_;
  }
  
  double Environment::getCollisionThreshold() const
  {
    return collision_thresh_;
  }
  
  bool Environment::getCost(const int x,
                            const int y,
                            double& cost) const
  {
    if (x >= x_size_ ||
      y >= y_size_ ||
      x < 0 ||
      y < 0)
      {
      return false;
      }
    
    cost = map_[y*x_size_ + x];
    return true;
  }
  
  double* Environment::getHeuristicMap() const
  {
    return heuristic_map_;
  }
  
  double* Environment::getForwardSearchMap() const
  {
    return forward_search_map_;
  }
  
  double Environment::getHeuristicCost(const int x,
                                       const int y) const
  {
    return heuristic_map_[y*x_size_ + x];
  }
  
  bool Environment::setHeuristicCost(const int x,
                                     const int y,
                                     double cost)
  {
    if (x >= x_size_ ||
      y >= y_size_ ||
      x < 0 ||
      y < 0)
      {
      return false;
      }
    
    heuristic_map_[y*x_size_ + x] = cost;
    return true;
  }
  
  bool Environment::setForwardSearchCost(const int x, const int y, double cost)
  {
    if (x >= x_size_ ||
        y >= y_size_ ||
        x < 0 ||
        y < 0)
      {
      return false;
      }
    
    forward_search_map_[y*x_size_ + x] = cost;
    return true;
  }
  
  Dijkstra::Dijkstra(EnvironmentPtr& environment_ptr) :
    environment_ptr_(environment_ptr)
  {
  }
  
  std::vector<Vertex2DPtr> Dijkstra::getValidSuccessors(const Vertex2DPtr& current_vertex)
  {
    std::vector<Vertex2DPtr> successors;
    
    for (int x = current_vertex->state_->x_ - 1;
         x <= current_vertex->state_->x_ + 1;
         x++)
      {
      for (int y = current_vertex->state_->y_ - 1;
           y <= current_vertex->state_->y_ + 1;
           y++)
        {
        
        if (x == current_vertex->state_->x_ &&
            y == current_vertex->state_->y_)
          {
          continue;
          }
        
        double successor_cost;
        if (!environment_ptr_->getCost(x, y, successor_cost))
          {
          continue;
          }
        
        if (successor_cost >= environment_ptr_->getCollisionThreshold())
          {
          continue;
          }
        
        State2DPtr successor_state = std::make_shared<State2D>(x, y);
        if (explored_.find(successor_state) == explored_.end())
          {
          Vertex2DPtr successor =
            std::make_shared<Vertex2D>(successor_state);
          successors.push_back(successor);
          explored_[successor_state] = successor;
          }
        else
          {
//          std::cout << "already in explored " << std::endl;
           successors.push_back(explored_[successor_state]);
          }
        }
      }
    return successors;
  }
  
  bool Dijkstra::getCost(const Vertex2DPtr &current_vertex,
                         const Vertex2DPtr &successor,
                         double& cost)
  {
    double successor_cost;
    if (!environment_ptr_->getCost(successor->state_->x_,
                                   successor->state_->y_,
                                   successor_cost))
      {
      return false;
      }
    cost = current_vertex->key_ +
           successor_cost + Params::TRANSITION_COST;
    return true;
  }
  
  bool Dijkstra::run(std::vector<State2DPtr>& start_states,
                     bool search_forward)
  {
    for (int i = 0; i < start_states.size(); ++i)
      {
      Vertex2DPtr start_node;
      start_node = std::make_shared<Vertex2D>(start_states[i]);
      start_node->root_id_ = i;
      start_node->depth_ = 0;
      explored_[start_states[i]] = start_node;
      open_.insert(start_node);
      open_.decreaseKey(start_node, 0.0);

        environment_ptr_->setForwardSearchCost(start_node->state_->x_,
                                               start_node->state_->y_,
                                               0.0);
      }

#if DEBUG
    int num_expansions = 0;
#endif

    while(open_.size() > 0)
      {

      Vertex2DPtr least_cost_vertex =
        std::static_pointer_cast<Vertex2D>(open_.remove());
      
      closed_[least_cost_vertex->state_] = true;

#if DEBUG
      ++num_expansions;
#endif
      
      std::vector<Vertex2DPtr> least_cost_vertex_successors =
        getValidSuccessors(least_cost_vertex);
      
      for(const Vertex2DPtr &successor : least_cost_vertex_successors)
        {
        if(closed_.find(successor->state_) == closed_.end())
          {
          double new_cost;
          if (!getCost(least_cost_vertex, successor, new_cost))
            {
            continue;
            }

          if (new_cost < successor->key_)
            {
            successor->parent_vertex_ = least_cost_vertex;
            successor->root_id_ = least_cost_vertex->root_id_;
            successor->depth_ = successor->parent_vertex_->depth_ + 1;
            if (successor->heap_index_ == -1)
              {
              open_.insert(successor);
              open_.decreaseKey(successor,
                                new_cost);
              }
            else
              {
              open_.decreaseKey(successor,
                                new_cost);
              }


              environment_ptr_->setForwardSearchCost(successor->state_->x_,
                                                     successor->state_->y_,
                                                     new_cost);

              environment_ptr_->setForwardSearchCost(successor->state_->x_,
                                                     successor->state_->y_,
                                                     new_cost);

            }
          }
        }
      }

#if DEBUG
    std::cout << "Total number of expansions: " << num_expansions << std::endl;
#endif
    
    return true;
  }
  
  Vertex2DPtr Dijkstra::stateToVertex(State2DPtr &state)
  {
    return explored_[state];
  }

  ForwardSearch::ForwardSearch(int robot_pose_x,
                               int robot_pose_y,
                               EnvironmentPtr& environment_ptr) : robot_pose_x_(robot_pose_x),
                                                                  robot_pose_y_(robot_pose_y),
                                                                  planner_(environment_ptr)
  {
  }
  
  bool ForwardSearch::run()
  {
    std::vector<State2DPtr> start_state;
    
    start_state.emplace_back(new State2D(robot_pose_x_,
                                         robot_pose_y_));
  
    using namespace std::chrono;
  
    auto start_time = Clock::now();
    if (planner_.run(start_state, true))
      {
      auto end_time = Clock::now();
      std::cout << "Forward search time: "
                << duration_cast<duration<double> >(end_time - start_time).count() << "s" << std::endl;
      return true;
      }
  
    return false;
  }

} // namespace homotopy_planner


