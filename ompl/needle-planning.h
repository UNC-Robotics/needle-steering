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

#ifndef OMPL_NEEDLE_NEEDLE_PLANNING_H
#define OMPL_NEEDLE_NEEDLE_PLANNING_H

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <string>
//#include <ompl/control/planners/rrt/RRT.h>
#include "NeedleRRT.h"
#include <ompl/config.h>

#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <ompl/NeedleStateSpace.h>
#include <ompl/NeedleDirectedControlSampler.h>
#include <ompl/NeedleGoalRegion.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/StateStorage.h>
#include "NeedleUtils.h"
#include "Environment.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

class NeedlePlanning
{
 public:

  NeedlePlanning(const Environment& env,
		 const double radius,
		 double nrad,
     double goalBias = 0.05,
		 double maxStepLen = 0.001);

  ~NeedlePlanning();

  ob::StateStoragePtr planFromPointToPoint(const Eigen::Vector3d startPos,
					   const Eigen::Matrix3d startRotation,
					   const Eigen::Vector3d goalPoint,
					   std::vector<Eigen::Vector3d>& resultPoints,
					   std::vector<Eigen::Matrix3d>& resultRots,
					   unsigned *result,
					   double duration);

  bool inReachableVolume(const Eigen::Vector3d startPos, const Eigen::Matrix3d startRotation, 
        const Eigen::Vector3d goalPoint);

  void writePathsToVisualizer(const std::vector<std::vector<Eigen::Vector3d> >& paths, const double needleRad,
        std::string outFile = "../../web-visualization/js/needle/needleCyls.js");

  void writeGoalToVisualizer(const Eigen::Vector3d& goal, const double rad,
        std::string outFile = "../../web-visualization/js/needle/goalSphere.js");


 private:
  const Environment& env_;
  const double radius_;
  const ob::State *start_;
  double maxStepLen_;
  double nrad_;
  bool checkCols_ = true;
  double goalBias_;

  Eigen::Matrix3d startRotation_;

  oc::SpaceInformationPtr si_;
  ob::PlannerPtr planner_;
  ob::StateSpacePtr space_;
  oc::ControlSpacePtr cspace_;

  bool isStateValid(const ob::State *state);

  // void propagate(const ob::State *start, const oc::Control *control,
	// 	 const double duration, ob::State *result);

  bool checkEdgeValid(const ob::State *s1, const ob::State *s2);

  oc::DirectedControlSamplerPtr NeedleDirectedControlSamplerAllocator(const oc::SpaceInformation *si)
    {
      return oc::DirectedControlSamplerPtr(new NeedleDirectedControlSampler(si, env_, radius_, nrad_,
									    checkCols_));
    }

};

#endif
