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

#include <ompl/base/goals/GoalRegion.h>
//#include "NeedleUtils.h"
namespace ob = ompl::base;
namespace oc = ompl::control;

class NeedleGoalRegion : public ompl::base::GoalSampleableRegion
{
 public:
  Eigen::Vector3d goalPoint_;
 NeedleGoalRegion(const ob::SpaceInformationPtr &si, Eigen::Vector3d goalPoint) :
  ompl::base::GoalSampleableRegion(si),
    goalPoint_(goalPoint)
  {
    //setThreshold(0.0001);//setThreshold(0.001);
  }

  void sampleGoal(ob::State *st) const
  {
    ob::SE3StateSpace::StateType *se3state = st->as<ob::SE3StateSpace::StateType>();
    double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    pos[0] = goalPoint_[0];
    pos[1] = goalPoint_[1];
    pos[2] = goalPoint_[2];
  }
  
  unsigned int maxSampleCount() const
  {
    return 1;
  }

  double distanceGoal(const ob::State *st) const
  {
    const ob::SE3StateSpace::StateType *se3state = st->as<ob::SE3StateSpace::StateType>();
    const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    //double dist = sqrt((pos[0] - goalPoint_[0])^2 + (pos[1] - goalPoint_[1])^2 + (pos[2] - goalPoint_[2])^2);
    //double dist = (pos[0] - goalPoint_[0])^2;
    Eigen::Vector3d point(pos[0], pos[1], pos[2]);
    double dist = (point - goalPoint_).norm();
    return dist;
  }
};
