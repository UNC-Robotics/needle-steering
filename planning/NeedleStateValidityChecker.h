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

#ifndef OMPL_NEEDLE_STATE_VALIDITY_CHECKER_H
#define OMPL_NEEDLE_STATE_VALIDITY_CHECKER_H

#include <ompl/base/SpaceInformation.h>
#include <Eigen/Dense>
#include <iostream> 

namespace ob = ompl::base;

class NeedleStateValidityChecker : public ob::StateValidityChecker
{
public:
  NeedleStateValidityChecker(const ob::SpaceInformationPtr &si) :
    ob::StateValidityChecker(si)
  {
      //May need to set startRot not in constructor but prior to starting planning.
  }
  virtual bool isValid(const ob::State *state) const
  {
    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
    double qw = se3state->rotation().w;
    double qx = se3state->rotation().x;
    double qy = se3state->rotation().y;
    double qz = se3state->rotation().z;

    //const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    Eigen::Quaternion<double> quatQ(qw,qx,qy,qz);
    Eigen::Matrix3d queryRot = quatQ.matrix();
    Eigen::Vector3d queryZ = queryRot.block<3,1>(0,2);
    Eigen::Vector3d startZ = startRotation_.block<3,1>(0,2);
    //Eigen::AngleAxis<double> queryAngle(quatQ);
    //Eigen::AngleAxis<double> startAngle(startRotation_);
    //Eigen::Vector3d vectQ(queryAngle);

    //std::cout << "DOT PRODUCT" << std::endl << startZ.dot(queryZ) << std::endl;
    if (startZ.dot(queryZ) < 0) return false;
    return true;
  }

  void setStartRotation(const Eigen::Matrix3d rot) {
    startRotation_ = rot;
  }
private:
  Eigen::Matrix3d startRotation_;
};

#endif