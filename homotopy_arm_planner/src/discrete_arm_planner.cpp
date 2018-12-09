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
 * \file   discrete_arm_planner.cpp
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   12/3/18
 */

#include <discrete_arm_planner.h>


namespace homotopy_planner
{

DiscreteArmPlanner::DiscreteArmPlanner(double *map,
                                     int x_size,
                                     int y_size,
                                     double *armstart_anglesV_rad,
                                     double *armgoal_anglesV_rad) : map_(map),
                                                                    x_size_(x_size),
                                                                    y_size_(y_size),
                                                                    num_DOFs_(static_cast<int>(NUM_DOF)),
                                                                    armstart_anglesV_rad_(new int[NUM_DOF]),
                                                                    armgoal_anglesV_rad_(new int[NUM_DOF])
{
  for (int i=0; i < num_DOFs_; ++i)
  {
    armstart_anglesV_rad_[i] = contToDisc(armstart_anglesV_rad[i]);
    armgoal_anglesV_rad_[i] = contToDisc(armgoal_anglesV_rad[i]);
  }
}

int DiscreteArmPlanner::contToDisc(double q) const
{
  q = normalize_angle_positive(q);
  int disc_q = static_cast<int>(q*DOUBLE_TO_INT_FACTOR);

  int step_size = static_cast<int>(DISCRETIZATION*DOUBLE_TO_INT_FACTOR);
  int disc_err = disc_q%step_size;
  disc_q = disc_err < step_size/2?
           disc_q - disc_err:
           disc_q + (step_size-disc_err);
  return disc_q;
}

double DiscreteArmPlanner::discToCont(int q) const
{
  q = normalize_angle_positive(q);
  return static_cast<double>(q)/DOUBLE_TO_INT_FACTOR;
}

double DiscreteArmPlanner::normalize_angle_positive(double angle) const
{
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

int DiscreteArmPlanner::normalize_angle_positive(int angle) const
{
  int discrete_pi = contToDisc(M_PI);
  return ((angle%(2*discrete_pi))+2*discrete_pi)%(2*discrete_pi);
}

double DiscreteArmPlanner::shortest_angular_distance(double angle) const
{
  double a = fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
  if (a > M_PI)
  {
    a = 2*M_PI - a;
  }
  return a;
}

int DiscreteArmPlanner::shortest_angular_distance(int angle) const
{
  int discrete_pi = contToDisc(M_PI);
  int a = ((angle%(2*discrete_pi))+2*discrete_pi)%(2*discrete_pi);
  if (a > discrete_pi)
  {
    a = 2*discrete_pi - a;
  }
  return a;
}

double DiscreteArmPlanner::distanceBetweenVertices(const homotopy_planner::VertexPtr &vertex_1,
                                                  const homotopy_planner::VertexPtr &vertex_2) const
{
  distanceBetweenVertices(vertex_1->state_.q_, vertex_2->state_.q_);
}

double DiscreteArmPlanner::distanceBetweenVertices(const homotopy_planner::ArmState &state_1,
                                                  const homotopy_planner::ArmState &state_2) const
{
  distanceBetweenVertices(state_1.q_, state_2.q_);
}

double DiscreteArmPlanner::distanceBetweenVertices(const int *q_1, const int *q_2) const
{
//  double weight[] = {500.0, 300.0, 200.0, 100.0, 5.0};
  double weight[] = {1.0, 1.0, 1.0, 1.0, 1.0};
  double squared_distance = 0.0;
  for (int i = 0; i < num_DOFs_; ++i)
  {
    squared_distance += weight[i] * std::pow(shortest_angular_distance(q_2[i] - q_1[i]),2);
//    squared_distance += weight[i] * std::pow(state_2.q_[i] - state_1.q_[i],2);
  }
  return std::sqrt(squared_distance);
}

void DiscreteArmPlanner::ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) const
{
  double cellsize = 1.0;
  //take the nearest cell
  *pX = (int)(x/(double)(cellsize));
  if( x < 0) *pX = 0;
  if( *pX >= x_size) *pX = x_size-1;

  *pY = (int)(y/(double)(cellsize));
  if( y < 0) *pY = 0;
  if( *pY >= y_size) *pY = y_size-1;
}


void DiscreteArmPlanner::get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) const
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
    params->Y1=p1x;
    params->X1=p1y;
    params->Y2=p2x;
    params->X2=p2y;
  }
  else
  {
    params->X1=p1x;
    params->Y1=p1y;
    params->X2=p2x;
    params->Y2=p2y;
  }

  if ((p2x - p1x) * (p2y - p1y) < 0)
  {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void DiscreteArmPlanner::get_current_point(bresenham_param_t *params, int *x, int *y) const
{
  if (params->UsingYIndex)
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  }
  else
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

int DiscreteArmPlanner::get_next_point(bresenham_param_t *params) const
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}



int DiscreteArmPlanner::IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
                                          int x_size,
                                          int y_size) const

{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  //make sure the line segment is inside the environment
  if(x0 < 0 || x0 >= x_size ||
     x1 < 0 || x1 >= x_size ||
     y0 < 0 || y0 >= y_size ||
     y1 < 0 || y1 >= y_size)
    return 0;

  ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
  ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

  //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  //iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do {
    get_current_point(&params, &nX, &nY);
    if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
      return 0;
  } while (get_next_point(&params));

  return 1;
}

int DiscreteArmPlanner::IsValidArmConfiguration(int* angles, int numofDOFs, double*	map,
                                               int x_size, int y_size) const
{
  double cont_angles[numofDOFs];
  for (int i=0; i<numofDOFs; ++i)
  {
    cont_angles[i] = discToCont(angles[i]);
  }

  //Self collision
  int roundoff = 100000;
  int link_starts[numofDOFs][2];
  int link_ends[numofDOFs][2];

  double x0,y0,x1,y1;
  int i;

  //iterate through all the links starting with the base
  x1 = ((double)x_size)/2.0;
  y1 = 0;
  for(i = 0; i < numofDOFs; i++)
  {
    //compute the corresponding line segment
    x0 = x1;
    y0 = y1;
    x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-cont_angles[i]);
    y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-cont_angles[i]);

    // Self collision
    link_starts[i][0] = static_cast<int>(x0*roundoff);
    link_starts[i][1] = static_cast<int>(y0*roundoff);
    link_ends[i][0] = static_cast<int>(x1*roundoff);
    link_ends[i][1] = static_cast<int>(y1*roundoff);

    //check the validity of the corresponding line segment
    if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
    {
      return 0;
    }

  }

#if AVOID_SELF_COLLISION
  if (!inSelfCollision(link_starts, link_ends))
  {
    return 0;
  }
#endif

  return 1;
}

// Self collision
bool DiscreteArmPlanner::inSelfCollision(int link_starts[][2], int link_ends[][2]) const
{

  for (int i = 0; i < num_DOFs_; ++i)
  {
    for (int j = 0; j < num_DOFs_; ++j)
    {
      if (i != j && i != j+1 && i != j-1)
      {
        // Find the four orientations needed for general and
        // special cases
        int o1 = lineSegmentOrientation(link_starts[i], link_ends[i], link_starts[j]);
        int o2 = lineSegmentOrientation(link_starts[i], link_ends[i], link_ends[j]);
        int o3 = lineSegmentOrientation(link_starts[j], link_ends[j], link_starts[i]);
        int o4 = lineSegmentOrientation(link_starts[j], link_ends[j], link_ends[i]);

        // General case
        if (o1 != o2 && o3 != o4)
        {
          return true;
        }

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && pointOnLineSegment(link_starts[i], link_starts[j], link_ends[i]))
        {
          return true;
        }
        // p1, q1 and q2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && pointOnLineSegment(link_starts[i], link_ends[j], link_ends[i]))
        {
          return true;
        }

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && pointOnLineSegment(link_starts[j], link_starts[i], link_ends[j]))
        {
          return true;
        }

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && pointOnLineSegment(link_starts[j], link_ends[i], link_ends[j]))
        {
          return true;
        }
      }
    }
  }

  return false; // Doesn't fall in any of the above cases
}

int DiscreteArmPlanner::lineSegmentOrientation(int *p, int *q, int *r) const
{
  int val = (q[1] - p[1]) * (r[0] - q[0]) -
            (q[0] - p[0]) * (r[1] - q[1]);

  if (val == 0)
  {
    return 0;  // colinear
  }

  return (val > 0)? 1: 2; // clock or counterclock wise
}

int DiscreteArmPlanner::pointOnLineSegment(int *p, int *q, int *r) const
{
  if (q[0] <= std::max(p[0], r[0]) &&
      q[0] >= std::min(p[0], r[0]) &&
      q[1] <= std::max(p[1], r[1]) &&
      q[1] >= std::min(p[1], r[1]))
  {
    return true;
  }

  return false;
}



} // namespace homotopy_planner
