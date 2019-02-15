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

#include "NeedleStateSpace.h"
#include "NeedleUtils.h"

NeedleStateSpace::NeedleStateSpace(double nrad) :
  nrad_(nrad)
{

}

double NeedleStateSpace::distance(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  //std::cout << "DISTANCE CALLED" << std::endl;
  Eigen::Matrix4d state1 = NeedleUtils::make4state(s1);
  Eigen::Matrix4d state2 = NeedleUtils::make4state(s2);
  Eigen::Vector3d point = state1.block<3,1>(0,3);
  //  std::cout << "Point2 = " << state2.block<3,1>(0,3) << std::endl;
  Eigen::Vector3d proj_point = state2.block<3,3>(0,0).transpose() * (point - state2.block<3,1>(0,3));
  double z = proj_point[2];
  //std::cout << "z = " << z << std::endl;
  if (z < 0) {
    //std::cout << "less than 0 is z" << std::endl;
    //std::cout << "returning INFTY"<< std::endl;
    return INFTY;
  }
  double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
  if (x == 0) {
    //std::cout << "returning " << z << std::endl;
    return z;
  }
  double r = (x*x + z*z) / (2*x);
  if (r < nrad_) { /////////////
    //std::cout << "returning INFTY"  << std::endl;
    return INFTY;
  }
  //std::cout << "returning " << atan2(z, r-x) * r << std::endl;
  return atan2(z, r-x) * r;
}

/*
Eigen::Matrix4d NeedleStateSpace::make4state(const ob::State* s) const
{
  const ob::SE3StateSpace::StateType *se3state = s->as<ob::SE3StateSpace::StateType>();
  const double* pos  = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
  const double qX    = se3state->as<ob::SO3StateSpace::StateType>(1)->x;
  const double qY    = se3state->as<ob::SO3StateSpace::StateType>(1)->y;
  const double qZ    = se3state->as<ob::SO3StateSpace::StateType>(1)->z;
  const double qW    = se3state->as<ob::SO3StateSpace::StateType>(1)->w;

  Eigen::Quaternion<double> quat(qW,qX,qY,qZ);
  Eigen::Matrix3d rotMat = quat.toRotationMatrix();

  Eigen::Matrix4d state = Eigen::Matrix4d::Identity();
  state.block(0,0,3,3) << rotMat;

  state(0,3) = pos[0];
  state(1,3) = pos[1];
  state(2,3) = pos[2];
  return state;
}
*/
