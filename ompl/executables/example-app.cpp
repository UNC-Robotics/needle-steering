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

#include <iostream>
#include "../needle-planning.h"

int main(int argc, char** argv) {
  ompl::base::StateStoragePtr str;

  Environment env;
  double radius = 0.001;
  double nrad = 0.121;
  double goalBias = 0.005;
  double needleRad = 0.0005;
  NeedlePlanning plan(env, radius, nrad, goalBias);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position(0,0,0);

  Eigen::Vector3d goal(0.01,0.01, 0.1);

  std::vector<std::vector<Eigen::Vector3d> > plans;

  int numRuns = 10;

  for (int i = 0; i < numRuns; ++i) {
    std::vector<Eigen::Vector3d> resultPos;
    std::vector<Eigen::Matrix3d> resultRots;
    unsigned result;
    plan.planFromPointToPoint(position, rotation, goal, resultPos, resultRots, &result, 10);
    std::cout << "result = " << result << std::endl;
    std::cout << "size of pos = " << resultPos.size() << std::endl;
    for (int i = 0; i < resultPos.size(); ++i) {
      // std::cout << resultPos.at(i).transpose() << std::endl;
      //std::cout << resultRots.at(i) << std::endl;
    }
    plans.push_back(resultPos);
  }
  plan.writePathsToVisualizer(plans, needleRad);
  plan.writeGoalToVisualizer(goal, 0.001);
  return 0;
}
