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

#include "needle-planning.h"
#include "NeedleStateValidityChecker.h"
#include "NeedleStatePropagator.h"

#include <ompl/util/Console.h>
#include <iostream>
#include <fstream>

//#include <cannula/cannula-utils/cannula-collisions.h>

NeedlePlanning::NeedlePlanning(const Environment& env,
			       const double radius,
			       double nrad,
             double goalBias,
			       double maxStepLen) :
  env_(env),
  radius_(radius),
  nrad_(nrad),
  goalBias_(goalBias),
  maxStepLen_(maxStepLen)
{
  //setuptime:

  //seed and annoying log
  ompl::RNG::setSeed(1);
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

  // construct the state space we are planning in
  space_ = ob::StateSpacePtr(new NeedleStateSpace(nrad_));

  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, -1.0);
  bounds.setLow(1, -1.0);
  bounds.setLow(2, -1.0);
  bounds.setHigh(0, 1.0);
  bounds.setHigh(1, 1.0);
  bounds.setHigh(2, 1.0);

  space_->as<ob::SE3StateSpace>()->setBounds(bounds);

  // create a control space
  cspace_ = oc::ControlSpacePtr(new oc::RealVectorControlSpace(space_, 3));

  // set the bounds for the control space
  ob::RealVectorBounds cbounds(3);

  //length
  cbounds.setLow(0, 0.05);
  cbounds.setHigh(0, 0.05);

  //rot
  cbounds.setLow(1, -3.0);
  cbounds.setHigh(1, 3.0);

  //rad
  cbounds.setLow(2, 0.001);
  cbounds.setHigh(2, 20.0);

  cspace_->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

  // construct an instance of  space information from this control space
  si_= oc::SpaceInformationPtr(new oc::SpaceInformation(space_, cspace_));

  // set state validity checking for this space
  si_->setStateValidityChecker(std::make_shared<NeedleStateValidityChecker>(si_));
  si_->setStatePropagator(std::make_shared<NeedleStatePropagator>(si_));
  si_->setDirectedControlSamplerAllocator(std::bind(&NeedlePlanning::NeedleDirectedControlSamplerAllocator,
						      this, std::placeholders::_1));
}

NeedlePlanning::~NeedlePlanning()
{

}

// bool NeedlePlanning::isStateValid(const ob::State *state)
// {
//   //std::cout << "isStateValid called " << std::endl;
//   //return true;////////////////////////////////////
//   const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
//   double qw = se3state->rotation().w;
//   double qx = se3state->rotation().x;
//   double qy = se3state->rotation().y;
//   double qz = se3state->rotation().z;

//   //const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
//   Eigen::Quaternion<double> quatQ(qw,qx,qy,qz);
//   Eigen::Matrix3d queryRot = quatQ.matrix();
//   Eigen::Vector3d queryZ = queryRot.block<3,1>(0,2);
//   Eigen::Vector3d startZ = startRotation_.block<3,1>(0,2);
//   //Eigen::AngleAxis<double> queryAngle(quatQ);
//   //Eigen::AngleAxis<double> startAngle(startRotation_);
//   //Eigen::Vector3d vectQ(queryAngle);

//   //std::cout << "DOT PRODUCT" << std::endl << startZ.dot(queryZ) << std::endl;
//   if (startZ.dot(queryZ) < 0) return false;
//   return true;
// }

// void NeedlePlanning::propagate(const ob::State *start, const oc::Control *control,
// 			       const double duration, ob::State *result)
// {
//   const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
//   double len = u[0];
//   double rot = u[1];
//   double rad = u[2];

//   double phi = (len/rad)*duration;

//   double ca = cos(rot);
//   double sa = sin(rot);

//   double cp = cos(phi);
//   double sp = sin(phi);

//   Eigen::Vector3d pt(rad * sa * (1.0 - cp), -rad * ca * (1.0 - cp), rad * sp);
//   Eigen::Vector3d z(sa * sp, -ca * sp, cp);
//   Eigen::Vector3d y(-sa * cp, ca * cp, sp);
//   Eigen::Vector3d x(ca, sa, 0);

//   Eigen::Matrix4d tU;
//   tU.block(0,0,3,1) <<  x;
//   tU.block(0,1,3,1) <<  y;
//   tU.block(0,2,3,1) <<  z;
//   tU.block(0,3,3,1) << pt;
//   tU(3,0) = 0.0; tU(3,1) = 0.0; tU(3,2) = 0.0; tU(3,3) = 1.0;

//   const ob::SE3StateSpace::StateType *se3state = start->as<ob::SE3StateSpace::StateType>();
//   const double* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0)->values;
//   const double qX    = se3state->as<ob::SO3StateSpace::StateType>(1)->x;
//   const double qY    = se3state->as<ob::SO3StateSpace::StateType>(1)->y;
//   const double qZ    = se3state->as<ob::SO3StateSpace::StateType>(1)->z;
//   const double qW    = se3state->as<ob::SO3StateSpace::StateType>(1)->w;

//   Eigen::Quaternion<double> quat(qW,qX,qY,qZ);
//   Eigen::Matrix3d rotMat = quat.toRotationMatrix();

//   Eigen::Matrix4d state = Eigen::Matrix4d::Identity();
//   state.block(0,0,3,3) << rotMat;
//   state(0,3) = pos[0];
//   state(1,3) = pos[1];
//   state(2,3) = pos[2];

//   Eigen::Matrix4d newState = state * tU;

//   result->as<ob::SE3StateSpace::StateType>()->setXYZ(
//                                                      newState(0,3),
//                                                      newState(1,3),
//                                                      newState(2,3));

//   Eigen::Matrix3d newRot = newState.block(0,0,3,3);
//   Eigen::Quaternion<double> newQuat(newRot);

//   ob::SE3StateSpace::StateType *newS = result->as<ob::SE3StateSpace::StateType>();
//   newS->as<ob::SO3StateSpace::StateType>(1)->x = newQuat.x();
//   newS->as<ob::SO3StateSpace::StateType>(1)->y = newQuat.y();
//   newS->as<ob::SO3StateSpace::StateType>(1)->z = newQuat.z();
//   newS->as<ob::SO3StateSpace::StateType>(1)->w = newQuat.w();

// }
bool NeedlePlanning::inReachableVolume(const Eigen::Vector3d startPos, const Eigen::Matrix3d startRotation, 
      const Eigen::Vector3d goalPoint) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3,3>() = startRotation;

  T(0,3) = startPos[0];
  T(1,3) = startPos[1];
  T(2,3) = startPos[2];

  Eigen::Vector3d proj_point = T.block<3,3>(0,0).transpose() * (goalPoint - T.block<3,1>(0,3));
  double z = proj_point[2];
  if (z < 0) {
    return false;
  }

  double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
  if (x == 0.0) {
    std::cout << "Does not allow straight travel" << std::endl;
    return false;/////Does not allow straight travel
  }
  else
  {
    double r = (x*x + z*z) / (2*x);
    if (r < nrad_)
    {
      //std::cout << "goal is out of reachable space" << std::endl;
      //std::cout << "r = " << r << std::endl;
      return false;
    }
    else
    {
      //std::cout << "REACHABLE" << std::endl;
      //std::cout << "r = " << r << std::endl;
      return true;
    }
  }
}

void NeedlePlanning::writePathsToVisualizer(const std::vector<std::vector<Eigen::Vector3d> >& paths, const double needleRad, 
      std::string outFile) {
  std::ofstream out;
  out.open(outFile);
  if (!out.is_open()) {
    std::cout << "Unable to open paths file." << std::endl;
  }
  int numPaths = paths.size();

  out << "var needleCyls = [";
  for (int i = 0; i < numPaths; ++i) {
    std::vector<Eigen::Vector3d> p = paths.at(i);
    
    out << "[";
    unsigned numPts = p.size();
    for (int j = 0; j < numPts; ++j) {
      out << "[";
      Eigen::Vector3d pt = p.at(j);
      out << pt(0) << ", " << pt(1) << ", " << pt(2) << ", " << needleRad;
      out << "]";
      if (j != numPts - 1) out << ", ";
    }
    out << "]";
    if (i != numPaths - 1) out << ", ";
    out << std::endl;
  }
  out << "];" << std::endl;
  out.close();
}

void NeedlePlanning::writeGoalToVisualizer(const Eigen::Vector3d& goal, const double rad,
        std::string outFile) {
  std::ofstream out;
  out.open(outFile);
  if (!out.is_open()) {
    std::cout << "Unable to open goal file." << std::endl;
  }

  out << "var sphere = ";

  out << "[";
  out << goal(0) << ", " << goal(1) << ", " << goal(2) << ", " << rad;
  out << "];";

  out.close();
}

ob::StateStoragePtr NeedlePlanning::planFromPointToPoint(const Eigen::Vector3d startPos,
							 const Eigen::Matrix3d startRotation,
							 const Eigen::Vector3d goalPoint,
							 std::vector<Eigen::Vector3d>& resultPoints,
							 std::vector<Eigen::Matrix3d>& resultRots,
							 unsigned *result,
							 double duration)
{
  ob::StateStoragePtr str;
  startRotation_ = Eigen::Matrix3d(startRotation);
  if (!inReachableVolume(startPos, startRotation, goalPoint)) {
    std::cout << "Outside of reachable volume." << std::endl;
    *result = 3;
    return str;
  }
  ob::StateValidityCheckerPtr valid = si_->getStateValidityChecker();
  std::shared_ptr<NeedleStateValidityChecker> nValid = std::dynamic_pointer_cast<NeedleStateValidityChecker>(valid);
  nValid->setStartRotation(startRotation_);
  ob::ScopedState<ob::SE3StateSpace> start(space_);
  //std::cout << "DURATION = " << duration << std::endl;
  start->setX(startPos[0]);
  start->setY(startPos[1]);
  start->setZ(startPos[2]);

  start->rotation().setIdentity();
  //Eigen::Quaternion<double> quat(Eigen::AngleAxisd(1, Eigen::Vector3d(0, 0, 1)));
  Eigen::Quaternion<double> quat(startRotation);
  quat.normalize();
  start->rotation().x = quat.x();
  start->rotation().y = quat.y();
  start->rotation().z = quat.z();
  start->rotation().w = quat.w();
  Eigen::Vector4d vv(quat.x(), quat.y(), quat.z(), quat.w());

  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_));

  // set the start and goal states
  pdef->addStartState(start);

  // create a planner for the defined space
  ob::PlannerPtr planner(new oc::NeedleRRT(si_));

  planner->as<oc::NeedleRRT>()->setGoalBias(goalBias_);
  planner->as<oc::NeedleRRT>()->setIntermediateStates(false);

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  pdef->setGoal(ob::GoalPtr(new NeedleGoalRegion(si_, goalPoint)));

  // print the settings for this space
  //si_->printSettings(std::cout);

  // print the problem settings
  //pdef->print(std::cout);

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();

  ob::PlannerStatus solved;

  solved = planner->solve(duration);
  //std::cout << "solved = " << solved << std::endl;

  if (solved == ob::PlannerStatus::TIMEOUT)
  {
    //std::cout << "TIMEOUT" << std::endl;
    (*result) = 0;
  }
  else if (solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
  {
    //std::cout << "APPROXIMATE_SOLUTION" << std::endl;
    (*result) = 1;
  }
  else if (solved == ob::PlannerStatus::EXACT_SOLUTION)
  {
    //std::cout << "EXACT_SOLUTION" << std::endl;
    (*result) = 2;
  }

  ob::PlannerData data(si_);

  planner->getPlannerData(data);

  str = data.extractStateStorage();

  ob::PathPtr path = pdef->getSolutionPath();
  // print the path to screen
  path->print(std::cout);

  unsigned int numStates = str->size();

  oc::PathControl* p = dynamic_cast<oc::PathControl*>(path.get());

  if (p != NULL)
  {
    std::vector<ob::State*> states = p->getStates();
    std::vector<oc::Control*> controls = p->getControls();
    for (int i = 0; i < states.size() - 1; ++i)
    {
      ob::State *st = states.at(i);
      oc::Control *c = controls.at(i);
      double* u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
      double oldL = u[0];
      int steps = 1;
      if (u[0] > maxStepLen_)
      {
        ob::State* tmpDest = si_->allocState();
        steps = (int) (u[0] / maxStepLen_) + 1;
        for (int i = 0; i < steps; ++i)
        {
          u[0] = maxStepLen_ * i;
          oc::StatePropagatorPtr prop = si_->getStatePropagator();
          //NeedlePlanning::propagate(st, c, 1.0, tmpDest);
          prop->propagate(st, c, 1.0, tmpDest);
          //print position and control
          double x = tmpDest->as<ob::SE3StateSpace::StateType>()->getX();
          double y = tmpDest->as<ob::SE3StateSpace::StateType>()->getY();
          double z = tmpDest->as<ob::SE3StateSpace::StateType>()->getZ();
          //f << x << " " << y << " " << z << " " << u[0]<< " " <<  u[1] << " " <<  u[2] << std::endl;
          //std::cout << "u[0] = " << u[0] << std::endl;
          resultPoints.push_back(Eigen::Vector3d(x,y,z));

          ob::SE3StateSpace::StateType *se3state = tmpDest->as<ob::SE3StateSpace::StateType>();
          double qX    = se3state->as<ob::SO3StateSpace::StateType>(1)->x;
          double qY    = se3state->as<ob::SO3StateSpace::StateType>(1)->y;
          double qZ    = se3state->as<ob::SO3StateSpace::StateType>(1)->z;
          double qW    = se3state->as<ob::SO3StateSpace::StateType>(1)->w;

          Eigen::Quaternion<double> quat1(qW,qX,qY,qZ);
          Eigen::Matrix3d rotMat = quat1.toRotationMatrix();
          resultRots.push_back(rotMat);
        }

	      si_->freeState(tmpDest);
	      u[0] = oldL;
      }
    }

    //have to do last state manually
    ob::State *st = states.at(states.size() - 1);
    double x = st->as<ob::SE3StateSpace::StateType>()->getX();
    double y = st->as<ob::SE3StateSpace::StateType>()->getY();
    double z = st->as<ob::SE3StateSpace::StateType>()->getZ();
    //f << x << " " << y << " " << z << " 0 0 0" << std::endl;
    resultPoints.push_back(Eigen::Vector3d(x,y,z));

    ob::SE3StateSpace::StateType *se3state = st->as<ob::SE3StateSpace::StateType>();
    double qX    = se3state->as<ob::SO3StateSpace::StateType>(1)->x;
    double qY    = se3state->as<ob::SO3StateSpace::StateType>(1)->y;
    double qZ    = se3state->as<ob::SO3StateSpace::StateType>(1)->z;
    double qW    = se3state->as<ob::SO3StateSpace::StateType>(1)->w;

    Eigen::Quaternion<double> quat2(qW,qX,qY,qZ);
    Eigen::Matrix3d rotMat = quat2.toRotationMatrix();
    resultRots.push_back(rotMat);
  }
  return str;
}

bool NeedlePlanning::checkEdgeValid(const ob::State* s1, const ob::State* s2)
{
  Eigen::Matrix4d T = NeedleUtils::make4state(s1);
  Eigen::Vector3d point = NeedleUtils::make4state(s2).block<3,1>(0,3);
  //std::cout << "point = " << std::endl << point << std::endl;
  Eigen::Vector3d proj_point = T.block<3,3>(0,0).transpose() * (point - T.block<3,1>(0,3));
  double z = proj_point[2];
  if (z < 0) {
    //std::cout << "BOOOOOSH" << std::endl;
    return false;
  }

  double x = sqrt(proj_point[0]*proj_point[0] + proj_point[1]*proj_point[1]);
  if (x == 0) {
    return false;/////Does not allow straight travel
  }
  double r = (x*x + z*z) / (2*x);
  if (r < nrad_)//0.05) //UGH 3 PLACES NOW GROSSSSSSSS
  {
    return false;
  }
  return true;
}
