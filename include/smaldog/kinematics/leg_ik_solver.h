/*
 * Copyright (c) 2014 Michael E. Ferguson.  All right reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */

#ifndef SMALDOG_KINEMATICS_LEG_IK_SOLVER_H_
#define SMALDOG_KINEMATICS_LEG_IK_SOLVER_H_

#include <math.h>

namespace smaldog
{

template<bool FRONT, bool RIGHT>
class LegIKSolver
{
public:
  /**
   *  \brief Constructor for IK solver of a single leg of the robot.
   *  \param name Name of the leg.
   *  \param shoulder Length of the shoulder link (in meters).
   *  \param femur Length of the femur link (in meters).
   *  \param tibia Length of the tibia link (in meters).
   */
  LegIKSolver(std::string name, double shoulder, double femur, double tibia)
   : name_(name), l_shoulder_(shoulder), l_femur_(femur), l_tibia_(tibia)
  {
  }

  /** \brief Get the name of the leg */
  std::string getName()
  {
    return name_;
  }

  /**
   *  \brief Compute the joint angles needed to hit a desired leg pose.
   *  \param x The x-coordinate of the desired leg pose (in meters).
   *  \param y The y-coordinate of the desired leg pose (in meters).
   *  \param z The z-coordinate of the desired leg pose (in meters).
   *  \param pitch The computed angle for the shoulder pitch joint (in radians).
   *  \param flex The computed angle for the shoulder flex joint (in radians).
   *  \param knee The computed angle for the knee joint (in radians).
   */
  bool solveIK(const double x, const double y, const double z,
               double& pitch, double& flex, double& knee) const
  {
    /* Solve shoulder pitch first, so that flex/knee form a 2d problem */
    double im1 = sqrt(y*y + z*z);
    double trueZ = sqrt(im1*im1 - l_shoulder_*l_shoulder_);
    double q1 = atan2(y, -z);
    double q2 = l_shoulder_/im1;
    if (!inRangeACos(q2))
      return false;
    if (RIGHT)
      pitch = q1 + acos(-q2) - M_PI/2.0;
    else
      pitch = q1 + acos(q2) - M_PI/2.0;
    
    /* Solve for femur angle from vertical */
    double im2 = sqrt(x*x + trueZ*trueZ);
    double q3;
    if (FRONT)
      q3 = atan2(-x, trueZ);
    else
      q3 = atan2(x, trueZ);
    double d1 = l_femur_*l_femur_ - l_tibia_*l_tibia_ + im2*im2;
    double d2 = 2 * l_femur_ * im2;
    if (!inRangeACos(d1/d2))
      return false;
    flex = q3 + acos(d1/d2);
    if (!FRONT)
      flex = -flex;

    /* And now, tibia angle from femur */
    double d3 = l_femur_*l_femur_ - im2*im2 + l_tibia_*l_tibia_;
    double d4 = 2 * l_tibia_ * l_femur_;
    if (!inRangeACos(d3/d4))
      return false;
    knee = acos(d3/d4) - M_PI;
    if (!FRONT)
      knee = -knee;

    return true;
  }

private:
  bool inRangeACos(const double x) const
  {
    return (x <= 1.0 && x >= -1.0);
  }

  /** \brief Name of the leg, for instance, "rf" */
  std::string name_;

  /** \brief Length of leg components (in meters) */
  double l_shoulder_, l_femur_, l_tibia_;
};

}  // namespace smaldog

#endif  // SMALDOG_KINEMATICS_LEG_IK_SOLVER_H_
