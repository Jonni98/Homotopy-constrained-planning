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

#ifndef PDR_HW_1_HASH_FUNCTIONS_H
#define PDR_HW_1_HASH_FUNCTIONS_H

// homotopy_planner
#include <heap.h>
#include <config.h>

// STL
#include <cstdio>
#include <vector>
#include <limits>
#include <boost/functional/hash.hpp>

namespace homotopy_planner
{

struct ArmState;
typedef Heap<double> PriorityQueue;
struct Vertex;

typedef std::shared_ptr<ArmState> ArmStatePtr;
typedef std::shared_ptr<Vertex> VertexPtr;


// FIXME Vertex should inherit from PriorityQueue::Element and not the ArmState.
// FIXME That was actually the main purpose of seprating out Vertex from the ArmState.
struct ArmState
{
  ArmState() : q_(new int[NUM_DOF])
  {
  }

  ArmState(int* joint_angles,
           std::vector<int> signature = std::vector<int>()) : q_(new int[NUM_DOF]),
                                                              signature_(std::move(signature))
  {
    std::copy(joint_angles, joint_angles + static_cast<int>(NUM_DOF), q_);
  }

//  ArmState(const ArmState &state) : q_(new int[NUM_DOF])
//  {
//    memcpy(q_, state.q_, static_cast<size_t>(NUM_DOF));
//  }

  int* q_;

  std::vector<int> signature_;
};

struct ArmStateEqual
{
  bool operator()(const ArmState &lhs, const ArmState &rhs) const
  {
    bool equal = true;
    for (int i=0; i<NUM_DOF; ++i)
    {
      equal = equal && (lhs.q_[i] == rhs.q_[i]);
    }
    return equal;
  }
};

struct Vertex : public PriorityQueue::Element
{
  Vertex(ArmState state, double g_cost=0.0) : state_(state),
                                              g_cost_(g_cost),
                                              parent_(nullptr),
                                              depth_(0)
  {
    key_ = std::numeric_limits<double>::infinity();
  }

  ~Vertex()
  {
  }

  int depth_;

  double g_cost_;

  VertexPtr parent_;

  ArmState state_;
};

} // namespace homotopy_planner


namespace std
{
  template <>
  struct hash<homotopy_planner::ArmState>
  {
    std::size_t operator()(const homotopy_planner::ArmState &state) const
    {
      std::size_t hash=0;
      for (int i=0; i<NUM_DOF; ++i)
      {
        boost::hash_combine(hash, state.q_[i]);
      }
#if DEBUG
//      std::cout<< "hash: " << hash<<std::endl;
#endif
      return hash;

//      return static_cast<std::size_t>((73856093 * state.q_[0]) ^
//                                      (19349663 * state.q_[1]) ^
//                                      (2654435761 * state.q_[2]));
    }
  };
} // namespace std

#endif // PDR_HW_1_HASH_FUNCTIONS_H
