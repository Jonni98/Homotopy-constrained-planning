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
 * \file   hash_functions.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.com)
 * \date   September 2018
 */

// homotopy_planner
#include <heap.h>

// STL
#include <cstdio>
#include <vector>
#include <limits>

namespace homotopy_planner
{

struct State2D;

typedef Heap<double> PriorityQueue;
typedef std::shared_ptr<State2D> State2DPtr;

struct State2D : public PriorityQueue::Element
{
  State2D(int x,
          int y) : x_(x),
                   y_(y)
  {
    key_ = std::numeric_limits<double>::infinity();
  }
  
  bool operator==(const State2D &rhs) const
  {
    return (x_ == rhs.x_ &&
            y_ == rhs.y_);
  }
  
  int x_;
  int y_;
};

struct State2DPtrEqual
{
  bool operator()(const State2DPtr& a,
                  const State2DPtr& b) const
  {
    return a->x_ == b->x_ &&
           a->y_ == b->y_;
  }
};

} // namespace homotopy_planner


namespace std
{
  
  template <>
  struct hash<homotopy_planner::State2D>
  {
    std::size_t operator()(const homotopy_planner::State2D &state) const
    {
//      return static_cast<std::size_t>((std::hash<int>()(state.x_)) ^
//                                      (std::hash<int>()(state.y_) << 1));
      return static_cast<std::size_t>((73856093 * state.x_) ^
                                      (19349663 * state.y_));
    }
    
  };
  
  template <> struct hash<homotopy_planner::State2DPtr>
  {
    std::size_t operator()(const homotopy_planner::State2DPtr& state_ptr) const
    {
      return hash<homotopy_planner::State2D>()(*state_ptr);
    }
  };
  
} // namespace std
