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

#ifndef SMALDOG_KINEMATICS_KINEMATICS_SOLVER_H_
#define SMALDOG_KINEMATICS_KINEMATICS_SOLVER_H_

#include <math.h>
#include <kdl/frames.hpp>
#include <smaldog/robot_state.h>
#include <smaldog/kinematics/leg_fk_solver.h>
#include <smaldog/kinematics/leg_ik_solver.h>

namespace smaldog
{

enum Legs
{
  LEFT_FRONT,
  RIGHT_REAR,
  RIGHT_FRONT,
  LEFT_REAR
};

/**
 *  \brief This is the whole body kinematics solver.
 */
class KinematicsSolver
{
public:
  /**
   *  \brief Constructor for the whole body kinematics solver.
   *  \param body_width Dimension measured between axis of rotation of the
   *         shoulder pitch servos (in meters).
   *  \param body_length Dimension measured between axis of rotation of the
   *         shoulder flex servos (in meters).
   *  \param shoulder Length of the shoulder link (in meters).
   *  \param femur Length of the femur link (in meters).
   *  \param tibia Length of the tibia link (in meters).
   */
  KinematicsSolver(double body_width, double body_length,
                   double shoulder, double femur, double tibia)
    : lf_ik_solver("lf", shoulder, femur, tibia),
      rr_ik_solver("rr", shoulder, femur, tibia),
      rf_ik_solver("rf", shoulder, femur, tibia),
      lr_ik_solver("lr", shoulder, femur, tibia),
      lf_fk_solver("lf", shoulder, femur, tibia),
      rr_fk_solver("rr", shoulder, femur, tibia),
      rf_fk_solver("rf", shoulder, femur, tibia),
      lr_fk_solver("lr", shoulder, femur, tibia),
      body_width_(body_width),
      body_length_(body_length)
  {
    
  }

  /**
   *  \brief Solve the forward kinematics.
   *  \param lf_foot_pose The pose of the left front foot in body_link frame.
   *  \param rr_foot_pose The pose of the right rear foot in body_link frame.
   *  \param rf_foot_pose The pose of the right front foot in body_link frame.
   *  \param lr_foot_pose The pose of the left rear foot in body_link frame.
   *  \param r The desired body roll.
   *  \param p The desired body pitch.
   *  \param y The desired body yaw.
   *  \param state The robot state, joint_positions and odom_transform will
   *         be updated if this is successful.
   *  \returns True if IK succeeds, false otherwise.
   */
  bool solveIK(const KDL::Vector& lf_foot_pose,
               const KDL::Vector& rr_foot_pose,
               const KDL::Vector& rf_foot_pose,
               const KDL::Vector& lr_foot_pose,
               const double r, const double p, const double y,
               RobotState& state) const
  {
    /* Make sure joint_positions is sized appropriately */
    if (state.joint_positions.size() < 12)
      state.joint_positions.resize(12);

    /* Create rotation matrix */
    KDL::Rotation rotation = KDL::Rotation::RPY(r, p, y);

    /* Left front leg position is foot pose - shoulder position */
    KDL::Vector lf_pose(body_length_/2.0, body_width_/2.0, 0);
    lf_pose = rotation * lf_pose;
    lf_pose = lf_foot_pose - lf_pose;
    if (!lf_ik_solver.solveIK(lf_pose.x(), lf_pose.y(), lf_pose.z(),
         state.joint_positions[1], state.joint_positions[3], state.joint_positions[5]))
      return false;

    /* Right rear leg position is foot pose - shoulder position */
    KDL::Vector rr_pose(-body_length_/2.0, -body_width_/2.0, 0);
    rr_pose = rotation * rr_pose;
    rr_pose = rr_foot_pose - rr_pose;
    if (!rr_ik_solver.solveIK(rr_pose.x(), rr_pose.y(), rr_pose.z(),
         state.joint_positions[6], state.joint_positions[8], state.joint_positions[10]))
      return false;

    /* Right front leg position is foot pose - shoulder position */
    KDL::Vector rf_pose(body_length_/2.0, -body_width_/2.0, 0);
    rf_pose = rotation * rf_pose;
    rf_pose = rf_foot_pose - rf_pose;
    if (!rf_ik_solver.solveIK(rf_pose.x(), rf_pose.y(), rf_pose.z(),
         state.joint_positions[0], state.joint_positions[2], state.joint_positions[4]))
      return false;

    /* Left rear leg position is foot pose - shoulder position */
    KDL::Vector lr_pose(-body_length_/2.0, body_width_/2.0, 0);
    lr_pose = rotation * lr_pose;
    lr_pose = lr_foot_pose - lr_pose;
    if (!lr_ik_solver.solveIK(lr_pose.x(), lr_pose.y(), lr_pose.z(),
         state.joint_positions[7], state.joint_positions[9], state.joint_positions[11]))
      return false;

    /* Update odometry rotation */
    state.body_transform.M = rotation;

    return true;
  }

  bool solveFK(const RobotState& state,
               KDL::Vector& lf_foot_pose,
               KDL::Vector& rr_foot_pose,
               KDL::Vector& rf_foot_pose,
               KDL::Vector& lr_foot_pose) const
  {
    /* Make sure joint_positions is sized appropriately */
    if (state.joint_positions.size() < 12)
      return false;

    double x, y, z;

    /* Left front leg position is foot pose + shoulder position */
    lf_foot_pose = KDL::Vector(body_length_/2.0, body_width_/2.0, 0);
    if (!lf_fk_solver.solveFK(state.joint_positions[1],
                              state.joint_positions[3],
                              state.joint_positions[5],
                              x, y, z))
      return false;
    lf_foot_pose += KDL::Vector(x, y, z);
    
    /* Right rear leg position is foot pose + shoulder position */
    rr_foot_pose = KDL::Vector(-body_length_/2.0, -body_width_/2.0, 0);
    if (!rr_fk_solver.solveFK(state.joint_positions[6],
                              state.joint_positions[8],
                              state.joint_positions[10],
                              x, y, z))
      return false;
    rr_foot_pose += KDL::Vector(x, y, z);

    /* Right front leg position is foot pose + shoulder position */
    rf_foot_pose = KDL::Vector(body_length_/2.0, -body_width_/2.0, 0);
    if (!rf_fk_solver.solveFK(state.joint_positions[0],
                              state.joint_positions[2],
                              state.joint_positions[4],
                              x, y, z))
      return false;
    rf_foot_pose += KDL::Vector(x, y, z);

    /* Left rear leg position is foot pose + shoulder position */
    lr_foot_pose = KDL::Vector(-body_length_/2.0, body_width_/2.0, 0);
    if (!lr_fk_solver.solveFK(state.joint_positions[7],
                              state.joint_positions[9],
                              state.joint_positions[11],
                              x, y, z))
      return false;
    lr_foot_pose += KDL::Vector(x, y, z);

    return true;
  }

private:
  LegIKSolver<true, false>  lf_ik_solver;
  LegIKSolver<false, true>  rr_ik_solver;
  LegIKSolver<true, true>   rf_ik_solver;
  LegIKSolver<false, false> lr_ik_solver;

  LegFKSolver<true, false>  lf_fk_solver;
  LegFKSolver<false, true>  rr_fk_solver;
  LegFKSolver<true, true>   rf_fk_solver;
  LegFKSolver<false, false> lr_fk_solver;

  double body_width_;
  double body_length_;
};

}  // namespace smaldog

#endif  // SMALDOG_KINEMATICS_KINEMATICS_SOLVER_H_
