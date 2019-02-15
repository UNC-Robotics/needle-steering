// Software License Agreement (BSD-3-Clause)
//
// Copyright 2018 The University of North Carolina at Chapel Hill
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Alan Kuntz

#ifndef OMPL_NEEDLE_NEEDLESTATESPACE
#define OMPL_NEEDLE_NEEDLESTATESPACE

#include <iostream>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <Eigen/Dense>

#define INFTY 9e9;
namespace ob = ompl::base;

class NeedleStateSpace : public ompl::base::SE3StateSpace
{
 public:
  const double nrad_;

  NeedleStateSpace(double nrad);

  double distance(const ompl::base::State* s1, const ompl::base::State* s2) const;

  Eigen::Matrix4d make4state(const ob::State* s) const;

  bool hasSymmetricDistance() const
  {
    return false;
  }

  bool hasSymmetricInterpolate() const
  {
    return false;
  }

};

#endif

/*
#include <iostream>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "NeedleUtils.h"

#define INFTY 9e9;
namespace ob = ompl::base;
namespace oc = ompl::control;

class NeedleStateSpace : public ompl::base::SE3StateSpace
{
public:
  const static double nrad = 0.121;//0.22;//0.05;//0.0135;0.121 in paper////////////////////////Defined in three places..... bad times

  double distance(const ompl::base::State* s1, const ompl::base::State* s2) const
  {
    Eigen::Matrix4d state1 = NeedleUtils::make4state(s1);
    Eigen::Matrix4d state2 = NeedleUtils::make4state(s2);

    Eigen::Vector3d point = state1.block<3,1>(0,3);
    Eigen::Vector3d proj_point = state2.block<3,3>(0,0).transpose() * (point - state2.block<3,1>(0,3));
    double z = proj_point[2];
    if (z < 0) {
      return INFTY;
    }
    double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
    if (x == 0) {
      return z;
    }
    double r = (x*x + z*z) / (2*x);
    if (r < nrad) {
      return INFTY;
    }
    return atan2(z, r-x) * r;
  }

  bool hasSymmetricDistance () const
  {
    return false;
  }

  bool hasSymmetricInterpolate () const
  {
    return false;
  }
};
*/
