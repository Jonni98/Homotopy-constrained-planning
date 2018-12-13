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
 * \file   h_signature.cpp
 * \author Muhammad Suhail
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   12/11/18
 */

#include <h_signature.h>

namespace homotopy_planner
{

HSignature::HSignature(const DiscreteArmPlanner* planner_handle,
                       double *map,
                       int x_size,
                       int y_size) : planner_handle_(planner_handle)
{
  representative_points_ = findRepresentativePoints(x_size, y_size, map);
  
//  for (const auto& rep_point : representative_points_)
//  {
//    std::cout << rep_point.x << "\t" << rep_point.y << std::endl;
//  }
}

std::vector<cv::Point2f> HSignature::findRepresentativePoints(int x_size,
                                                              int y_size,
                                                              double *map_array)
{
  cv::Mat map(y_size, x_size, CV_8UC1,cv::Scalar(0));
  for(int y=0;y<y_size;y++)
  {
    for(int x=0;x<x_size;x++)
    {
      uchar color = map.at<uchar>(cv::Point(x,y));
      color = map_array[y*x_size+x]==0?0:255;
      map.at<uchar>(cv::Point(x,y)) = color;
    }
  }
  cv::Mat grayscaleMap;
  cv::Mat thr,thresh;
  cv::threshold(map, thr, 100,255,cv::THRESH_BINARY);
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(thr, contours,hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  std::vector<cv::Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  {
    mu[i] = moments( contours[i], false );
  }

  // get the centroid of figures.
  std::vector<cv::Point2f> mc(contours.size());
  for( int i = 0; i<contours.size(); i++)
  {
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    // cout<<mc<<'\n';
  }
  // for(int i=0;i<mc.size();i++)
  // cout<<mc[i]<<'\n';
  // imshow("hi",thr);
  // waitKey(0);
  return mc;
}

std::vector<std::string>  HSignature::generateSignatures(std::vector<cv::Point2f> representative_points)
{
  std::vector<int> symbols;
  std::vector<std::string> signatures;
  std::vector<std::string> signatures_of_iteration;
  for(int i=1;i<=representative_points.size();i++)
  {
    symbols.push_back(i);
    symbols.push_back(-i);

  }
  // for(int i=0;i<symbols.size();i++)
  // {
  //   cout<<symbols[i]<<'\n';
  // }
  for(int i = 0;i<symbols.size();i++)
  {
    signatures_of_iteration = permGenerator(symbols.size(), i + 1, symbols);
    for(int k =0;k<signatures_of_iteration.size();k++)
    {
      if(std::find(signatures.begin(), signatures.end(), signatures_of_iteration[k])==signatures.end())
        signatures.push_back(signatures_of_iteration[k]);
    }
  }
   for(int i=0;i<signatures.size();i++)
   {
     std::cout<< signatures[i] << std::endl;
   }
  return signatures;
}

std::vector<std::string> HSignature::permGenerator(int n,
                                                   int k,
                                                   std::vector<int> symbols)
{
  std::vector<int> d(n);
  std::vector<std::string> a;
  std::iota(d.begin(),d.end(),1);
  // cout << "These are the Possible Permutations: " << endl;
  int vec_num = 0;
  do
  {
    std::vector<int> b;
    for (int i = 0; i < k; i++)
    {
      b.push_back(symbols[d[i]-1]);
    }
    std::vector<int> c;
    c = computeReducedWord(b, k);
    std::string reducedword;
    for(int j=0;j<c.size();j++)
    {
      // cout<<c[j]<<' ';
      reducedword+=std::to_string(c[j]);
    }
    if(std::find(a.begin(), a.end(), reducedword)==a.end())
      a.push_back(reducedword);
    std::reverse(d.begin()+k,d.end());
  } while (next_permutation(d.begin(),d.end()));

  std::string word;
  return a;
}

std::vector<int> HSignature::computeReducedWord(std::vector<int> word,
                                                int k)
{
  std::vector<int> reduced_word;
  for(int i=0;i<k;i++)
  {
    if(std::find(word.begin(), word.end(), word[i]*-1)==word.end())
    {
      reduced_word.push_back(word[i]);
    }
  }
  return reduced_word;
}

std::vector<int> HSignature::updateSignature(const homotopy_planner::ArmState &from_state,
                                             const homotopy_planner::ArmState &to_state)
{
  // TODO Get rid of the redundant `current_signature` variable
  std::vector<int> current_signature = from_state.signature_;
  std::vector<int> updated_signature = current_signature;

  DiscreteArmPlanner::Point2D from_end_effector_pose =
          planner_handle_->getEndEffectorPose(from_state.q_, NUM_DOF);
  DiscreteArmPlanner::Point2D to_end_effector_pose =
          planner_handle_->getEndEffectorPose(to_state.q_, NUM_DOF);

  int sign = 0;
  for (const auto& rep_point : representative_points_)
  {
    sign++;
    if (from_end_effector_pose.y_ < rep_point.y ||
        to_end_effector_pose.y_ < rep_point.y)
    {
      if (from_end_effector_pose.x_ < rep_point.x &&
          to_end_effector_pose.x_ > rep_point.x)
      {
        if (current_signature.empty())
        {
          updated_signature.push_back(sign);
        }
        else if (current_signature.back() == -1*sign)
        {
          updated_signature.pop_back();
        }
        else
        {
          updated_signature.push_back(sign);
        }
      }
      else if (from_end_effector_pose.x_ > rep_point.x &&
               to_end_effector_pose.x_ < rep_point.x)
      {
        if (current_signature.empty())
        {
          updated_signature.push_back(-sign);
        }
        else if (current_signature.back() == sign)
        {
          updated_signature.pop_back();
        }
        else
        {
          updated_signature.push_back(-sign);
        }
      }
    }
  }

  return updated_signature;
}

int HSignature::computeSignatureMismatch(const std::vector<int> from_sign,
                                         const std::vector<int> to_sign) const
{
  int edit_distance = 0;
  
  int min_sign_length = std::min(from_sign.size(), to_sign.size());
  
  for (int i=0; i<min_sign_length; ++i)
  {
    if (from_sign[i] != to_sign[i])
    {
      ++edit_distance;
    }
  }
  edit_distance *= SIGN_CHAR_MISMATCH_PENALTY;
  
  edit_distance += std::abs(from_sign.size()-to_sign.size())*SIGN_LENGTH_MISMATCH_PENALTY;
  
  return edit_distance;
}

} // namespace homotopy_planner