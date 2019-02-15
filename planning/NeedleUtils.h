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

#ifndef OMPL_NEEDLE_NEEDLEUTILS
#define OMPL_NEEDLE_NEEDLEUTILS

#include <ompl/control/PathControl.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

//double nrad;// = 0.135;
//double insertRate;// = 1.0;// cm/second or whatever
//double replanTime;// = 1.0;// seconds or whatever

namespace NeedleUtils
{

  inline Eigen::Matrix4d make4state(const ob::State* s)
  {
    const ob::SE3StateSpace::StateType *se3state = s->as<ob::SE3StateSpace::StateType>();
    const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double qX    = se3state->as<ob::SO3StateSpace::StateType>(1)->x;
    const double qY    = se3state->as<ob::SO3StateSpace::StateType>(1)->y;
    const double qZ    = se3state->as<ob::SO3StateSpace::StateType>(1)->z;
    const double qW    = se3state->as<ob::SO3StateSpace::StateType>(1)->w;

    Eigen::Quaternion<double> quat(qW,qX,qY,qZ);
    Eigen::Matrix3d rotMat = quat.toRotationMatrix();

    Eigen::Matrix4d state = Eigen::Matrix4d::Identity();
    state.block(0,0,3,3) << rotMat;

    //std::cout << "rotmat = " << std::endl << rotMat << std::endl;
    state(0,3) = pos[0];
    state(1,3) = pos[1];
    state(2,3) = pos[2];
    return state;
  }

  inline double pathCost(ob::PathPtr path)
  {
    if (path == 0)
    {
      //std::cout << "null path means no solutions" << std::endl;
      return 10000000000000000.0;
    }
    double cost = 0.0;
    oc::PathControl* p = dynamic_cast<oc::PathControl*>(path.get());
    //std::cout << "p = " << p << std::endl;
    std::vector<oc::Control*> controls = p->getControls();
    for (int i = 0; i < controls.size(); ++i)
    {
      oc::Control *c = controls.at(i);
      double* u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
      cost += u[0];
    }
    return cost;
  }

  inline double pathMinClearance(std::vector<Eigen::Vector3d> needlePathPoints,
				 std::vector<Eigen::Matrix3d> needlePathRots)
  {
    int path = 0;
    if (path == 0)
    {
      //std::cout << "null path means no solutions" << std::endl;
      return 0.0;
    }
    double minClear = 0.0;
    return minClear;
  }

}

#endif
