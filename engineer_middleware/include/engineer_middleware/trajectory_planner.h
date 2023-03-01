//
// Created by lsy on 23-3-1.
//
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

namespace engineer_middleware
{
class Trajectory_planner
{
public:
  Trajectory_planner();
  ~Trajectory_planner();
  static void joints_cubic_polynomial(moveit::planning_interface::MoveGroupInterface& interface,
                                      moveit::planning_interface::MoveGroupInterface::Plan plan,
                                      std::vector<double> target)
  {
    // Define the start and end joint values
    std::vector<double> start_joint_values = interface.getCurrentJointValues();
    std::vector<double> end_joint_values = target;

    // Calculate the coefficients of the cubic polynomial for each joint
    std::vector<std::vector<double>> coefficients;
    for (int i = 0; i < (int)start_joint_values.size(); ++i)
    {
      double a0 = start_joint_values[i];
      double a3 = (2.0 * start_joint_values[i] - 2.0 * end_joint_values[i]);
      double a2 = (3.0 * end_joint_values[i] - 3.0 * start_joint_values[i]);
      double a1 = -a0 - a2 - a3;
      coefficients.push_back({ a0, a1, a2, a3 });
    }

    // Generate the trajectory
    for (double t = 0.0; t < 1.0; t += 0.01)
    {
      std::vector<double> joint_positions;
      std::vector<double> joint_velocities;
      std::vector<double> joint_accelerations;
      for (int i = 0; i < (int)coefficients.size(); ++i)
      {
        double position = coefficients[i][0] + coefficients[i][1] * t + coefficients[i][2] * pow(t, 2) +
                          coefficients[i][3] * pow(t, 3);
        double velocity = { coefficients[i][1] + 2 * coefficients[i][2] * t + 3 * coefficients[i][3] * pow(t, 2) };
        double acceleration = { 2 * coefficients[i][2] + 6 * coefficients[i][3] * t };
        joint_positions.push_back(position);
        joint_velocities.push_back(velocity);
        joint_accelerations.push_back(acceleration);
      }
      // header
      //            plan.trajectory_.joint_trajectory.header.seq = ;
      //            plan.trajectory_.joint_trajectory.header.stamp = ;
      plan.trajectory_.joint_trajectory.header.frame_id = "world";

      // joint_name
      plan.trajectory_.joint_trajectory.joint_names = interface.getJointNames();

      // position info
      plan.trajectory_.joint_trajectory.points.back().positions = joint_positions;
      plan.trajectory_.joint_trajectory.points.back().velocities = joint_velocities;
      plan.trajectory_.joint_trajectory.points.back().accelerations = joint_accelerations;
      plan.trajectory_.joint_trajectory.points.back().time_from_start = ros::Duration(t);
    }
  }
};

}  // namespace engineer_middleware
