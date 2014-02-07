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

#ifndef SMALDOG_KINEMATICS_LEG_FK_SOLVER_H_
#define SMALDOG_KINEMATICS_LEG_FK_SOLVER_H_

#include <math.h>

namespace smaldog
{

template<bool FRONT, bool RIGHT>
class LegFKSolver
{
public:
  /**
   *  \brief Constructor for FK solver of a single leg of the robot.
   *  \param name Name of the leg.
   *  \param shoulder Length of the shoulder link.
   *  \param femur Length of the femur link.
   *  \param tibia Length of the tibia link.
   */
  LegFKSolver(std::string name, double shoulder, double femur, double tibia)
   : name_(name), l_shoulder_(shoulder), l_femur_(femur), l_tibia_(tibia)
  {
    if (RIGHT)
      l_shoulder_ = -l_shoulder_;
  }

  /** \brief Get the name of the leg */
  std::string getName()
  {
    return name_;
  }

  /**
   *  \brief Compute the leg pose based on joint angles.
   *  \param pitch The angle for the shoulder pitch joint.
   *  \param flex The angle for the shoulder flex joint.
   *  \param knee The angle for the knee joint.
   *  \param x The computed x-coordinate of the leg pose.
   *  \param y The computed y-coordinate of the leg pose.
   *  \param z The computed z-coordinate of the leg pose.
   */
  bool solveFK(const double pitch, const double flex, const double knee,
               double& x, double& y, double&z)
  {
    /* Do the 2D math first */
    x = -sin(flex)*l_femur_ - sin(flex + knee)*l_tibia_;
    z = -cos(flex)*l_femur_ - cos(flex + knee)*l_tibia_;

    /* Now add shoulder pitch */
    y = cos(pitch)*l_shoulder_ - sin(pitch)*z;
    z = sin(pitch)*l_shoulder_ + cos(pitch)*z;

    return true;
  }

private:
  /** \brief Name of the leg, for instance, "rf" */
  std::string name_;

  /** \brief Length of leg components */
  double l_shoulder_, l_femur_, l_tibia_;
};

}  // namespace smaldog

#endif  // SMALDOG_KINEMATICS_LEG_FK_SOLVER_H_
