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

#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SpaceInformation.h>
//#include <cannula/cannula-utils/cannula-collisions.h>
#include <ompl/util/RandomNumbers.h>
#include "NeedleUtils.h"
#include "Environment.h"

namespace ob = ompl::base;
namespace oc = ompl::control;


class NeedleDirectedControlSampler : public ompl::control::DirectedControlSampler
{
 public:
  double maxStepLen_;
  const double nrad_;// = 0.121;//0.22;//0.05;//0.0135;//0.005;0.121 is in paper//////////////////////////
  const Environment& env_;
  const double radius_;
  const bool detectCols_;
  //double insertRate = 1.0;

  const ompl::control::StatePropagatorPtr  statePropagator_;
  NeedleDirectedControlSampler (const oc::SpaceInformation *si,
				const Environment& env, const double radius, const double nrad,
				const bool detectCols) :
  // NeedleDirectedControlSampler (const oc::SpaceInformation *si,
	// 			const double radius, const double nrad,
	// 			const bool detectCols) :
    DirectedControlSampler(si),
    statePropagator_(si->getStatePropagator()),
    maxStepLen_(0.001),
    env_(env),
    radius_(radius),
    nrad_(nrad),
    detectCols_(detectCols) {}

  unsigned int sampleTo (oc::Control *control, const ob::State *source, ob::State *dest)
  {
    double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::Matrix4d T = NeedleUtils::make4state(source);
    Eigen::Vector3d point = NeedleUtils::make4state(dest).block<3,1>(0,3);
    //std::cout << "point = " << std::endl << point << std::endl;
    Eigen::Vector3d proj_point = T.block<3,3>(0,0).transpose() * (point - T.block<3,1>(0,3));
    double z = proj_point[2];
    if (z < 0) {
      //std::cout << "BOOOOOSH" << std::endl;
      return 0;
    }

    double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
    if (x == 0) {
      return 0;/////Does not allow straight travel

      u[0] = z;
      u[1] = 0;
      u[2] = INFTY;
      return 1;
    }
    double r = (x*x + z*z) / (2*x);
    if (r < nrad_) {
      return 0;
    }

    // controls in terms of l,theta,r
    u[0] = atan2(z, r-x) * r;
    u[1] = atan2(proj_point[0], -proj_point[1]);
    u[2] = r;

    /////////////////////////////////
    ompl::RNG gen;///////////UGLY
    double rndm = gen.uniform01();
    double buff = 0.04 * rndm + 0.01;
    if (u[0] > buff) u[0] = buff;


    //std::cout << "rndm  " << rndm << std::endl;
    //u[0] = rndm * u[0];
    //std::cout << "u[0] = " << u[0] << std::endl;

    double oldL = u[0];

    int steps = 1;
    ob::State* tmpSrc  = si_->allocState();
    ob::State* tmpDest = si_->allocState();
    si_->copyState(tmpSrc, source);
    //    std::cout << "maxStepLen_ = " << maxStepLen_ << std::endl;
    //    std::cout << "u[0] = " << u[0] << std::endl;
    if (u[0] > maxStepLen_)
    {
      steps = (int) (u[0] / maxStepLen_) + 1;
      //std::cout << "steps = " << steps << std::endl;
      for (int i = 0; i < steps - 1; ++i)
      {
	//u[0] = maxStepLen_ * i;
	u[0] = maxStepLen_;
	statePropagator_->propagate(tmpSrc, control, 1.0, tmpDest);
	//if (!si_->isValid(tmpDest))
	if (detectCols_)
	{
	  if (!collisionFree(tmpSrc, tmpDest) || !si_->isValid(tmpDest))
	  {
	    si_->freeState(tmpSrc);
	    si_->freeState(tmpDest);
	    //std::cout << "COLLISION AAAAAH" << std::endl;
	    return 0;
	  }
	}
	else
	{
	  if (!si_->isValid(tmpDest))
	  {
	    si_->freeState(tmpSrc);
	    si_->freeState(tmpDest);
	    return 0;
	  }
	}
	si_->copyState(tmpSrc, tmpDest);
      }
      u[0] = oldL;
    }
    else
    {
      //do the thing once, make sure it's right
    }

    //si_->freeState(tmpSrc);
    //si_->freeState(tmpDest);
    statePropagator_->propagate(source, control, 1.0, dest);

    if (!si_->isValid(dest))
    {
      return 0;
    }

    return 1;
  }

  // This will just return true while I figure out how to handle non-binary probability of collision stuff.
  bool collisionFree(ob::State *source, ob::State *dest)
  {
    double x = source->as<ob::SE3StateSpace::StateType>()->getX();
    double y = source->as<ob::SE3StateSpace::StateType>()->getY();
    double z = source->as<ob::SE3StateSpace::StateType>()->getZ();
    Eigen::Vector3d srcPos(x,y,z);

    ob::SE3StateSpace::StateType *se3source = source->as<ob::SE3StateSpace::StateType>();
    double qX    = se3source->as<ob::SO3StateSpace::StateType>(1)->x;
    double qY    = se3source->as<ob::SO3StateSpace::StateType>(1)->y;
    double qZ    = se3source->as<ob::SO3StateSpace::StateType>(1)->z;
    double qW    = se3source->as<ob::SO3StateSpace::StateType>(1)->w;

    Eigen::Quaternion<double> quat1(qW,qX,qY,qZ);
    Eigen::Matrix3d srcRotMat = quat1.toRotationMatrix();

    double xD = dest->as<ob::SE3StateSpace::StateType>()->getX();
    double yD = dest->as<ob::SE3StateSpace::StateType>()->getY();
    double zD = dest->as<ob::SE3StateSpace::StateType>()->getZ();
    Eigen::Vector3d destPos(xD,yD,zD);

    ob::SE3StateSpace::StateType *se3dest = dest->as<ob::SE3StateSpace::StateType>();
    double qXD    = se3dest->as<ob::SO3StateSpace::StateType>(1)->x;
    double qYD    = se3dest->as<ob::SO3StateSpace::StateType>(1)->y;
    double qZD    = se3dest->as<ob::SO3StateSpace::StateType>(1)->z;
    double qWD    = se3dest->as<ob::SO3StateSpace::StateType>(1)->w;

    Eigen::Quaternion<double> quat2(qWD,qXD,qYD,qZD);
    Eigen::Matrix3d destRotMat = quat2.toRotationMatrix();

    //check for collisions using the environment class
    return env_.isNeedleSegmentCollisionFree(srcPos, destPos, srcRotMat, destRotMat, radius_);
    return true;
  }

  unsigned int sampleTo (oc::Control *control, const oc::Control *previous, const ob::State *source, ob::State *dest)
  {
    return sampleTo(control, source, dest);
  }

  //doesn't have collision detection, also not used currently, useful in replanning setting
  int maxCurveSampleTo (oc::Control *control, const ob::State *source, ob::State *dest)
  {
    double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::Matrix4d T = NeedleUtils::make4state(source);
    Eigen::Vector3d point = NeedleUtils::make4state(dest).block<3,1>(0,3);
    Eigen::Vector3d proj_point = T.block<3,3>(0,0).transpose() * (point - T.block<3,1>(0,3));
    double z = proj_point[2];
    if (z < 0) {
      //std::cout << "bad z" << std::endl;
      return 0;
    }

    double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
    if (x == 0) {
      //std::cout << "bad x" << std::endl;
      return 0;/////Remove if allowing straight travel

      u[0] = z;
      u[1] = 0;
      u[2] = INFTY;
      return 0;
    }
    double r = (x*x + z*z) / (2*x);
    if (r < nrad_) {
      //std::cout << "BAD R" << std::endl;
      r = nrad_;
    }

    // Output controls in terms of l,theta,r
    u[0] = atan2(z, r-x) * r;
    u[1] = atan2(proj_point[0], -proj_point[1]);
    u[2] = r;

    double oldL = u[0];
    int steps = 1;
    if (u[0] > maxStepLen_)
    {
      ob::State* tmpDest = si_->allocState();
      steps = (int) (u[0] / maxStepLen_) + 1;
      for (int i = 0; i < steps; ++i)
      {
	u[0] = maxStepLen_ * i;
	statePropagator_->propagate(source, control, 1.0, tmpDest);

	double x = tmpDest->as<ob::SE3StateSpace::StateType>()->getX();
	double y = tmpDest->as<ob::SE3StateSpace::StateType>()->getY();
	double z = tmpDest->as<ob::SE3StateSpace::StateType>()->getZ();
	//std::cout << "dst = " << x << "," << y << "," << z << std::endl;
	//std::cout << "steps = " << steps;
	if (!si_->isValid(tmpDest))
	{
	  //std::cout << "bad dst" << std::endl;
	  //dest->as<ob::SE3StateSpace::StateType>()->setXYZ(x,y,z);
	  return 0;
	}
      }
      si_->freeState(tmpDest);
      u[0] = oldL;
    }
    statePropagator_->propagate(source, control, 1.0, dest);

    if (!si_->isValid(dest))
    {
      //std::cout << "bad lastdest" << std::endl;
      return 0;
    }

    return 1;
  }
};
