/*
 * Copyright (c) 2014-2024 Michael E. Ferguson.  All right reserved.
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

#ifndef SMALDOG_STABILITY_COG_TRAJECTORY_H
#define SMALDOG_STABILITY_COG_TRAJECTORY_H

#include <math.h>

#include <kdl/frames.hpp>
#include <smaldog/robot_state.h>
#include <smaldog/kinematics/kinematics_solver.h>

namespace smaldog
{

inline KDL::Vector findProjectedIntersection(
  const KDL::Vector& a1, const KDL::Vector& a2,
  const KDL::Vector& b1, const KDL::Vector& b2)
{
  KDL::Vector intersection;

  double m_a = (a2.y() - a1.y())/(a2.x() - a1.x());
  double m_b = (b2.y() - b1.y())/(b2.x() - b1.x());

  double c_a = a2.y() - m_a * a2.x();
  double c_b = b2.y() - m_b * b2.x();

  if (!std::isfinite(m_a))
  {
    /* a is vertical */
    intersection.x(a1.x());
    intersection.y(m_b * intersection.x() + c_b);
  }
  else if (!std::isfinite(m_b))
  {
    /* b is vertical */
    intersection.x(b1.x());
    intersection.y(m_a * intersection.x() + c_a);
  }
  else
  {
    intersection.x((c_b-c_a) / (m_a-m_b));
    intersection.y(m_a * intersection.x() + c_a);
  }
  return intersection;
}

/**
 *  TODO: Update this description
 *  \brief Compute the fast quad shift for a particular robot state, as described
 *         in Pognas et. Al 2007.
 *  \param state The state of the robot joints and which legs are in contact
 *         with the ground.
 *  \param shift The computed shift. This represents a movement of the body COG
 *         within the body-centric coordinate frame.
 *  \returns true if shift is properly computed, false if there is some sort of
 *           error (typically numerical)
 */
inline bool computeCOGShift(const KDL::Vector& lf, const KDL::Vector& rr,
                            const KDL::Vector& rf, const KDL::Vector& lr,
                            const double t, KDL::Vector& shift)
{
  double alpha_lr = 0.25;
  double alpha_fb = 0.1;

  KDL::Vector xc = findProjectedIntersection(lf, rr, rf, lr);
  //std::cout << "xc   " << xc.x() << " " << xc.y() << " " << xc.z() << std::endl << std::endl;

  KDL::Vector a_lr = findProjectedIntersection(lf, lr, xc, xc + KDL::Vector(0, 10, 0)) - xc;
  //std::cout << "a_lr " << a_lr.x() << " " << a_lr.y() << " " << a_lr.z() << std::endl << std::endl;
  KDL::Vector a_fb = findProjectedIntersection(lf, rf, xc, xc + KDL::Vector(10, 0, 0)) - xc;
  //std::cout << "a_fb " << a_fb.x() << " " << a_fb.y() << " " << a_fb.z() << std::endl << std::endl;

  double sin_lr = sin(2 * M_PI * t + M_PI/2.0);
  double sin_fb = sin(4 * M_PI * t);

  //std::cout << sin_lr << std::endl;
  //std::cout << sin_fb << std::endl;

  shift.x(a_lr.x() * alpha_lr * sin_lr + a_fb.x() * alpha_fb * sin_fb);
  shift.y(a_lr.y() * alpha_lr * sin_lr + a_fb.y() * alpha_fb * sin_fb);
  shift.z(a_lr.z() * alpha_lr * sin_lr + a_fb.z() * alpha_fb * sin_fb);

  shift += xc;

  //std::cout << "x.append(" << shift.x() << "); y.append(" << shift.y() << ")" << std::endl;

  return true;
}

}  // namespace smaldog

#endif  // SMALDOG_STABILITY_COG_TRAJECTORY_H
