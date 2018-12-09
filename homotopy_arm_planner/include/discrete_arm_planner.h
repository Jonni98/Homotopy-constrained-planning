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
 * \file   discrete_arm_planner.h
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   12/3/18
 */

#ifndef PDR_HW_1_DISCRETE_ARM_PLANNER_H
#define PDR_HW_1_DISCRETE_ARM_PLANNER_H

// STL
#include <random>
#include <memory>
#include <ctime>
#include <iostream>
#include <chrono>
#include <fstream>


// Project
#include <config.h>
#include <macros.h>
#include <hash_functions.h>

namespace homotopy_planner
{

class DiscreteArmPlanner
{
public:
  typedef struct {
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
  } bresenham_param_t;

  DiscreteArmPlanner(double*	map,
                     int x_size,
                     int y_size,
                     double* armstart_anglesV_rad,
                     double* armgoal_anglesV_rad);

  int contToDisc(double q) const;

  double discToCont(int q) const;

  int shortest_angular_distance(int angle) const;

  int normalize_angle_positive(int angle) const;

  double shortest_angular_distance(double angle) const;

  double normalize_angle_positive(double angle) const;

  virtual double distanceBetweenVertices(const VertexPtr& vertex_1,
                                         const VertexPtr& vertex_2) const;

  virtual double distanceBetweenVertices(const ArmState& state_1,
                                         const ArmState& state_2) const;

  virtual double distanceBetweenVertices(const int* q_1,
                                         const int* q_2) const;

  void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) const;

  void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) const;

  void get_current_point(bresenham_param_t *params, int *x, int *y) const;

  int get_next_point(bresenham_param_t *params) const;

  int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
                         int x_size,
                         int y_size) const;

  int IsValidArmConfiguration(int* angles, int numofDOFs, double*	map,
                              int x_size, int y_size) const;

  // Self collision
  bool inSelfCollision(int link_starts[][2], int link_ends[][2]) const;

  int lineSegmentOrientation(int* p, int* q, int* r) const;

  int pointOnLineSegment(int* p, int* q, int* r) const;

protected:

  double*	map_;
  int x_size_;
  int y_size_;
  int* armstart_anglesV_rad_;
  int* armgoal_anglesV_rad_;
  int num_DOFs_;

};

} // namespace homotopy

#endif //PDR_HW_1_DISCRETE_ARM_PLANNER_H
