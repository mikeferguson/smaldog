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

namespace smaldog
{

/**
 *  \brief The walking controller. This forms the main node.
 */
class WalkingController
{
public:

  // add ability to select planner?

  bool update(ros::Time time)
  {
    // 1. update planner goals
    // 2. plan next step
    // 3. execute action
  }

private:
  /** \brief Callback for feedback from FollowJointTrajectoryAction */ 
  void actionFeedbackCallback(/* TODO */)
  {
    // publish COG/support polygon
    // must be done here by mixing current RobotState (for which feet are support) with the JointState feedback
    // publish odometry
  }

  /** \brief Callback for command velocity from higher level planner or controller */
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
  {
    // update goals to send to planner
  }

  // TODO connect to full body follow joint trajectory action
};

}  // namespace smaldog
