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
 * \file   h_signature.h
 * \author Muhammad Suhail
 * \author Ramkumar Natarajan (rnataraj@cs.cmu.edu)
 * \date   12/11/18
 */

#ifndef PDR_HW_1_H_SIGNATURE_H
#define PDR_HW_1_H_SIGNATURE_H

// STL
#include <iostream>
#include <vector>
#include <string>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

// Project
#include <hash_functions.h>
#include <discrete_arm_planner.h>

namespace homotopy_planner
{

class HSignature;

typedef std::shared_ptr<HSignature> HSignaturePtr;

class HSignature
{
public:

  HSignature(const DiscreteArmPlanner* planner_handle,
             double*	map,
             int x_size,
             int y_size);

  std::vector<cv::Point2f> findRepresentativePoints(int x_size, int y_size, double *map_array);

  std::vector<int> computeReducedWord(std::vector<int> word, int k);

  std::vector<std::string> permGenerator(int n, int k, std::vector<int> symbols);

  std::vector<std::string> generateSignatures(std::vector<cv::Point2f> representative_points);

  std::vector<int> updateSignature(const ArmState& from_state,
                                   const ArmState& to_state);

  int computeSignatureMismatch(const std::vector<int> from_sign,
                               const std::vector<int> to_sign) const;

private:

  const DiscreteArmPlanner* planner_handle_;

  std::vector<cv::Point2f> representative_points_;


};

} // namespace homotopy_planner

#endif //PDR_HW_1_H_SIGNATURE_H
