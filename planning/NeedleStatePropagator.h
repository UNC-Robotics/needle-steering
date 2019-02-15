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

#ifndef OMPL_NEEDLE_STATE_PROPAGATOR_H
#define OMPL_NEEDLE_STATE_PROPAGATOR_H

#include <ompl/control/StatePropagator.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class NeedleStatePropagator : public oc::StatePropagator {

public:
  NeedleStatePropagator(const oc::SpaceInformationPtr &si) :
    oc::StatePropagator(si)
  {
  }

  virtual void propagate(const ompl::base::State *start, const ompl::control::Control* control,
           const double duration, ompl::base::State *result) const {
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    double len = u[0];
    double rot = u[1];
    double rad = u[2];

    double phi = (len/rad)*duration;

    double ca = cos(rot);
    double sa = sin(rot);

    double cp = cos(phi);
    double sp = sin(phi);

    Eigen::Vector3d pt(rad * sa * (1.0 - cp), -rad * ca * (1.0 - cp), rad * sp);
    Eigen::Vector3d z(sa * sp, -ca * sp, cp);
    Eigen::Vector3d y(-sa * cp, ca * cp, sp);
    Eigen::Vector3d x(ca, sa, 0);

    Eigen::Matrix4d tU;
    tU.block(0,0,3,1) <<  x;
    tU.block(0,1,3,1) <<  y;
    tU.block(0,2,3,1) <<  z;
    tU.block(0,3,3,1) << pt;
    tU(3,0) = 0.0; tU(3,1) = 0.0; tU(3,2) = 0.0; tU(3,3) = 1.0;

    const ob::SE3StateSpace::StateType *se3state = start->as<ob::SE3StateSpace::StateType>();
    const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
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

    Eigen::Matrix4d newState = state * tU;

    result->as<ob::SE3StateSpace::StateType>()->setXYZ(
                                                      newState(0,3),
                                                      newState(1,3),
                                                      newState(2,3));

    Eigen::Matrix3d newRot = newState.block(0,0,3,3);
    Eigen::Quaternion<double> newQuat(newRot);

    ob::SE3StateSpace::StateType *newS = result->as<ob::SE3StateSpace::StateType>();
    newS->as<ob::SO3StateSpace::StateType>(1)->x = newQuat.x();
    newS->as<ob::SO3StateSpace::StateType>(1)->y = newQuat.y();
    newS->as<ob::SO3StateSpace::StateType>(1)->z = newQuat.z();
    newS->as<ob::SO3StateSpace::StateType>(1)->w = newQuat.w();
  }
private:

};

#endif