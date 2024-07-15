// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef NAV2_autodock_RECOVEY__autodock_RECOVERY_HPP_
#define NAV2_autodock_RECOVEY__autodock_RECOVERY_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_autodock_behavior/action/auto_dock.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_autodock_behavior/autodock_utils.hpp"

enum class ParallelCorrection : uint8_t
{
  START = 1,
  SPIN_RIGHT = 2,
  ON_HEADING = 3,
  SPIN_LEFT = 4,
  DONE = 5
};

namespace nav2_autodock_behavior
{

using namespace nav2_behaviors;  // NOLINT
using Action = nav2_autodock_behavior::action::AutoDock;

class AutoDock : public TimedBehavior<Action>
{
public:
  AutoDock();
  ~AutoDock();

  Status onRun(const std::shared_ptr<const Action::Goal> command) override;

  Status onCycleUpdate() override;

  void onConfigure() override;

  void publish_cmd(
    const double linear_vel,
    const double angular_vel);

  void get_center_of_side_markers(double offset);
  geometry_msgs::msg::PoseStamped output_center_of_side_markers(double offset);
  geometry_msgs::msg::PoseStamped get_tf(std::string target_link, std::string ref_link, rclcpp::Time target_time);
  geometry_msgs::msg::PoseStamped get_tf(std::string target_link);
  void set_state(DockState state);
  void set_state(DockState state,std::string printout);
  bool isCollisionFree(geometry_msgs::msg::Twist * cmd_vel, geometry_msgs::msg::Pose2D & pose2d);

protected:

  bool do_spin(double cmd_yaw);
  bool do_move(double cmd_dis, double command_speed);
  void do_predock();
  void do_parallel_correction();
  void do_steer_dock();
  void do_last_mile();

  std::vector<double>  linear_vel_range_; // min,max linear velocity
  std::vector<double>  angular_vel_range_; // min,max angular velocity
  // float tf_expiry;
  float simulate_ahead_time_;
  // float dock_timeout;
  float controller_rate_;
  
  // Frames and topics
  std::string base_link_;
  std::string left_marker_;
  std::string right_marker_;
  std::string center_marker_;
  geometry_msgs::msg::PoseStamped left_tf_;
  geometry_msgs::msg::PoseStamped right_tf_;
  geometry_msgs::msg::PoseStamped center_tf_;
  
  // Velocity settings
  float max_linear_vel_; // m/s, for parallel.c and steer
  float min_linear_vel_; // m/s, for lastmile
  float max_angular_vel_; // rad/s
  float min_angular_vel_; // rad/s

  // Predock stop yaw angle
  float stop_yaw_diff_; // rad
    
  // // Debug mode
  bool debug_mode_;

  double offset_to_angular_vel_; //    # factor to convert y-offset to ang vel
  double cam_offset_; // = 0.35           # camera to base link, for edge2edge distance
  double to_last_mile_dis_; // = 0.5      # distance to last mile
  double to_last_mile_tol_; // = 0.1       # tolerance to last mile
  double max_last_mile_odom_; // = 0.1     # max odom distance to last mile
  double max_parallel_offset_;
  double stop_distance_; //  # edge2edge distance to stop from charger
  double remaining_dis_;
  bool front_dock_; // = True
  bool use_sim_time_; // = False

  double offset_from_charger_;
  
  int dir_; // Direction of docking, 1 for front, -1 for back
  
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

  DockState dock_state_;
  double parallel_correction_offset_;
  double transition_dis_with_tol_;

  void set_parallel(ParallelCorrection state);
  void set_parallel(ParallelCorrection state, std::string printout);
  ParallelCorrection parallel_state_;
  double spin_relative_yaw_;
  double prev_yaw_;
  geometry_msgs::msg::PoseStamped move_initial_pose_;
};

}  // namespace nav2_autodock_recovery

#endif  // NAV2_autodock_RECOVEY__autodock_RECOVERY_HPP_
