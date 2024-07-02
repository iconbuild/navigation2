// Copyright (c) 2020 Samsung Research America
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <cmath>
#include <chrono>
#include <memory>

#include "nav2_autodock_behavior/autodock.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_autodock_behavior
{

AutoDock::AutoDock()
: TimedBehavior<Action>()
{
}

AutoDock::~AutoDock()
{
}

void AutoDock::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  
  // nav2_util::declare_parameter_if_not_declared(
  //   node,
  //   "tf_expiry", rclcpp::ParameterValue(1.0));
  // node->get_parameter("tf_expiry",tf_expiry_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "simulate_ahead_time_dock", rclcpp::ParameterValue(2.0));
  node->get_parameter("simulate_ahead_time_dock", simulate_ahead_time_);

  // nav2_util::declare_parameter_if_not_declared(
  //   node,
  //   "dock_timeout", rclcpp::ParameterValue(100.0));
  // node->get_parameter("dock_timeout",dock_timeout_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "controller_rate", rclcpp::ParameterValue(10.0));
  node->get_parameter("controller_rate",controller_rate_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "base_link", rclcpp::ParameterValue("base_link"));
  node->get_parameter("base_link",base_link_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "left_marker", rclcpp::ParameterValue("charge_left"));
  node->get_parameter("left_marker",left_marker_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "right_marker", rclcpp::ParameterValue("charge_right"));
  node->get_parameter("right_marker",right_marker_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "center_marker", rclcpp::ParameterValue("charge_center"));
  node->get_parameter("center_marker",center_marker_);
  
  std::vector<double> v = {-0.2,0.2};
  nav2_util::declare_parameter_if_not_declared(
    node,
    "linear_vel_range", rclcpp::ParameterValue(v));
  node->get_parameter("linear_vel_range",linear_vel_range_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "angular_vel_range", rclcpp::ParameterValue(v));
  node->get_parameter("angular_vel_range",angular_vel_range_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_linear_vel", rclcpp::ParameterValue(0.05));
  node->get_parameter("max_linear_vel",max_linear_vel_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_linear_vel", rclcpp::ParameterValue(0.02));
  node->get_parameter("min_linear_vel",min_linear_vel_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_angular_vel", rclcpp::ParameterValue(0.2));
  node->get_parameter("max_angular_vel",max_angular_vel_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "min_angular_vel", rclcpp::ParameterValue(0.05));
  node->get_parameter("min_angular_vel",min_angular_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "stop_yaw_diff", rclcpp::ParameterValue(0.03));
  node->get_parameter("stop_yaw_diff",stop_yaw_diff_);
  

  nav2_util::declare_parameter_if_not_declared(
    node,
    "debug_mode", rclcpp::ParameterValue(true));
  node->get_parameter("debug_mode",debug_mode_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "cam_offset", rclcpp::ParameterValue(0.35));
  node->get_parameter("cam_offset",cam_offset_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "offset_to_angular_vel", rclcpp::ParameterValue(2.0));
  node->get_parameter("offset_to_angular_vel",offset_to_angular_vel_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_parallel_offset", rclcpp::ParameterValue(0.5));
  node->get_parameter("max_parallel_offset",max_parallel_offset_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "to_last_mile_dis", rclcpp::ParameterValue(0.5));
  node->get_parameter("to_last_mile_dis",to_last_mile_dis_);
  
  nav2_util::declare_parameter_if_not_declared(
    node,
    "to_last_mile_tol", rclcpp::ParameterValue(0.1));
  node->get_parameter("to_last_mile_tol",to_last_mile_tol_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "stop_distance", rclcpp::ParameterValue(0.12));
  node->get_parameter("stop_distance",stop_distance_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "max_last_mile_odom", rclcpp::ParameterValue(0.1));
  node->get_parameter("max_last_mile_odom",max_last_mile_odom_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "front_dock", rclcpp::ParameterValue(true));
  node->get_parameter("front_dock",front_dock_);

  dir_ = (front_dock_) ? 1 : -1;

  offset_from_charger_ = cam_offset_ + to_last_mile_dis_;
  transition_dis_with_tol_ = offset_from_charger_ + to_last_mile_tol_;
  set_state(DockState::IDLE);

}

Status AutoDock::onRun(const std::shared_ptr<const Action::Goal> command)
{
  /**
   * @brief calls and runs the autodock action server
   * @param command : timeout length
   * @note this function runs once per call of the server
   * @return success or failure under timed behavior param
   */
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return Status::FAILED;
  }

  RCLCPP_INFO(logger_, "Start Docking Action Now");
  set_state(DockState::PREDOCK,"Predock");
  set_parallel(ParallelCorrection::START);
  spin_relative_yaw_ = 0.0;
  command_time_allowance_ = command->time_allowance;
  end_time_ = this->clock_->now() + command_time_allowance_;

  return Status::SUCCEEDED; 
}

Status AutoDock::onCycleUpdate()
{
  /**
   * @brief clock cycle update of the autodock action server
   * @param command : timeout length
   * @note this function runs once per call of the server
   * @return success or failure under timed behavior param
   */

  rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the autodock goal - Exiting AutoDock");
    return Status::FAILED;
  }

  
  switch(dock_state_){
    case DockState::IDLE:
      return Status{Status::SUCCEEDED};
    case DockState::INVALID:
      return Status{Status::FAILED};
    case DockState::PREDOCK:
      do_predock();
      break;
    case DockState::PARALLEL_CORRECTION:
      do_parallel_correction();
      break;
    case DockState::STEER_DOCK:
      do_steer_dock();
      break;
    case DockState::LAST_MILE:
      do_last_mile();
      break;
    case DockState::ACTIVATE_CHARGER:
      return Status{Status::SUCCEEDED};
      // do_activate_charger();
    case DockState::RETRY:
      // do_retry();
    case DockState::PAUSE:
      // do_pause();
    default:
      break;
  }
  return Status{Status::RUNNING};
}

void AutoDock::publish_cmd(double linear_vel=0.0,double angular_vel=0.0)
{
  // Publish the command to the topic
  // RCLCPP_INFO(node->get_logger(), "Publishing command");
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_vel;
  msg.angular.z = angular_vel;
  msg.linear.x = (msg.linear.x > linear_vel_range_[1]) ? linear_vel_range_[1] : msg.linear.x;
  msg.linear.x = (msg.linear.x < linear_vel_range_[0]) ? linear_vel_range_[0] : msg.linear.x;
  msg.angular.z = (msg.angular.z > angular_vel_range_[1] ) ? angular_vel_range_[1] : msg.angular.z;
  msg.angular.z = (msg.angular.z < angular_vel_range_[0]) ? angular_vel_range_[0] : msg.angular.z;
  vel_pub_->publish(std::move(msg));
}

void AutoDock::set_state(DockState state){
  dock_state_ = state;
}

void AutoDock::set_state(DockState state,std::string printout){
  dock_state_ = state;
  if (debug_mode_){
    RCLCPP_INFO(logger_,printout.c_str());
  }
}

void AutoDock::set_parallel(ParallelCorrection state){
  parallel_state_ = state;
}

void AutoDock::set_parallel(ParallelCorrection state, std::string printout){
  parallel_state_ = state;
  if (debug_mode_){
    RCLCPP_INFO(logger_,printout.c_str());
  }
}

geometry_msgs::msg::PoseStamped AutoDock::get_tf(std::string target_link, std::string ref_link, rclcpp::Time target_time)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (ref_link == ""){
    ref_link = robot_base_frame_;
  }
  if (target_time == rclcpp::Time(0)){
    target_time = this->clock_->now();
  }
  if (!nav2_util::getCurrentPose(current_pose, *tf_, ref_link, target_link,transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    geometry_msgs::msg::PoseStamped empty_pose;
    return empty_pose;
  }
  return current_pose;
}

geometry_msgs::msg::PoseStamped AutoDock::get_tf(std::string target_link)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, robot_base_frame_, target_link,transform_tolerance_))
  {
    // RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    geometry_msgs::msg::PoseStamped empty_pose;
    return empty_pose;
  }
  return current_pose;
}

void AutoDock::get_center_of_side_markers(double offset=0.0){
  /**
   * @brief Get the center of the two side markers
   * @return change to centre_tf
   */
  left_tf_ = get_tf(left_marker_);
  right_tf_ = get_tf(right_marker_);
  if (left_tf_.header.frame_id.empty() || right_tf_.header.frame_id.empty()){
    center_tf_ = geometry_msgs::msg::PoseStamped();
  }
  else{
    center_tf_ = autodock_util::get_center_tf(left_tf_, right_tf_, offset); //convert to center
  }
  // return left_tf;
}

geometry_msgs::msg::PoseStamped AutoDock::output_center_of_side_markers(double offset=0.0){
  /**
   * @brief Get the center of the two side markers
   * @return geometry_msgs::msg::PoseStamped : center of the two side markers
   */
  if (left_tf_.header.frame_id.empty()  || right_tf_.header.frame_id.empty()){
    return geometry_msgs::msg::PoseStamped();
  }
  else{
    return autodock_util::get_center_tf(left_tf_, right_tf_, offset); //convert left_tf to center
  }
}


void AutoDock::do_predock(){
  /**
   * @brief Function that is run when in the PREDOCK state
   * @details This function will run every cycle update when the state machine is in predock. Possible transitions lead to idle, failed and to steer_dock
  */
  get_center_of_side_markers();
  if (center_tf_.header.frame_id.empty()){
    RCLCPP_ERROR(logger_, "Not detecting two side markers, exit state");
    set_state(DockState::INVALID);
    return;
  }

  if (front_dock_){
    autodock_util::flip_base_frame(center_tf_);
  }
  double yaw = tf2::getYaw(center_tf_.pose.orientation);
  // if (debug_mode_){
  //   RCLCPP_WARN(logger_, "Dis %.3f | ", center_tf_.pose.position.x);
  //   RCLCPP_WARN(logger_, "Offset %.3f", center_tf_.pose.position.y);
  //   RCLCPP_WARN(logger_, "Yaw %.3f", yaw);
  // }
  if (std::fabs(yaw) < stop_yaw_diff_){
    if (debug_mode_){RCLCPP_INFO(logger_, "Done with yaw correction");}
    stopRobot();
    double y_offset = center_tf_.pose.position.y;
    if (std::fabs(y_offset) > max_parallel_offset_){
      parallel_correction_offset_ = y_offset;
      set_state(DockState::PARALLEL_CORRECTION);
      if (debug_mode_){RCLCPP_WARN(logger_, "Parallel correction %.3f", y_offset);}
      set_parallel(ParallelCorrection::START);
      return;
    }
    set_state(DockState::STEER_DOCK,"Steer dock.");
    return;
  }
  double ang_vel = autodock_util::bin_filter(yaw, min_angular_vel_);
  publish_cmd(0.0,ang_vel);
  return;
}

void AutoDock::do_parallel_correction(){
  /**
   * @brief Function that runs in state machine PARALLEL_CORRECTION each cycle. Utilizes side markers to correct the robot's orientation.
   * 
  */
  
  switch(parallel_state_){
    case ParallelCorrection::START:
      {spin_relative_yaw_ = 0.0;
      geometry_msgs::msg::PoseStamped current_pose;
      nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
      prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);
      set_parallel(ParallelCorrection::SPIN_RIGHT,"Start spin right");
      break;}
    case ParallelCorrection::SPIN_RIGHT:
      if (do_spin(-0.5*M_PI)){
        nav2_util::getCurrentPose(move_initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_);
        if (debug_mode_){RCLCPP_INFO(logger_,"Start move offset: %.3f",parallel_correction_offset_);}
        set_parallel(ParallelCorrection::ON_HEADING);
      }
      break;
    case ParallelCorrection::ON_HEADING:
      if (do_move(parallel_correction_offset_,max_linear_vel_)){
        spin_relative_yaw_ = 0.0;
        set_parallel(ParallelCorrection::SPIN_LEFT,"Start spin left");
      }
      break;
    case ParallelCorrection::SPIN_LEFT:
      if (do_spin(0.5*M_PI)){set_parallel(ParallelCorrection::DONE,"Parallel correction done");}
      break;
    case ParallelCorrection::DONE:
      parallel_correction_offset_ = 0.0;
      set_state(DockState::PREDOCK,"Restarting predock.");
      set_parallel(ParallelCorrection::START);
      break;
  }
  return;
}

bool AutoDock::do_spin(double cmd_yaw){
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    set_state(DockState::INVALID,"Current robot pose is not available.");
    return false;
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
  double delta_yaw = current_yaw - prev_yaw_;
  if (abs(delta_yaw) > M_PI) {
    delta_yaw = copysign(2 * M_PI - abs(delta_yaw), prev_yaw_);
  }

  spin_relative_yaw_ += delta_yaw;
  prev_yaw_ = current_yaw;
  double remaining_yaw = abs(cmd_yaw) - abs(spin_relative_yaw_);
  if (debug_mode_){RCLCPP_INFO(logger_,"Spin mode remaining yaw -> yaw: %.3f, remaining yaw: %.3f", spin_relative_yaw_,remaining_yaw);}
  if (remaining_yaw < 1e-6) {
    stopRobot();
    return true;
  }

  // Collision checking
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->angular.z = copysign(max_angular_vel_, cmd_yaw);
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(cmd_vel.get(), pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting Autodock");
    set_state(DockState::INVALID);
    return false;
  }

  publish_cmd(0.0,copysign(max_angular_vel_, cmd_yaw));
  return false;
}

bool AutoDock::do_move(double cmd_dis, double command_speed){
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    set_state(DockState::INVALID);
    return false;
  }

  double diff_x = move_initial_pose_.pose.position.x - current_pose.pose.position.x;
  double diff_y = move_initial_pose_.pose.position.y - current_pose.pose.position.y;
  double distance = hypot(diff_x, diff_y);
  if (distance > std::fabs(cmd_dis)) {
    stopRobot();
    return true;
  }

  //Collision checking
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;
  cmd_vel->linear.x = copysign(command_speed,cmd_dis);

  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(cmd_vel.get(), pose2d)) {
     this->stopRobot();
     RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
     set_state(DockState::INVALID);
     return false;
  }

  publish_cmd(copysign(command_speed,cmd_dis),0.0);
  return false;
}

void AutoDock::do_steer_dock(){
  /** 
   * @brief Function that runs in state machine STEER_DOCK each cycle. Utilizes side markers to steer the robot closer to the charging station.
   * 
  */
  get_center_of_side_markers(offset_from_charger_);
  // If sides are not detectable, look for center marker
  if (center_tf_.header.frame_id.empty()){
    RCLCPP_WARN(logger_, "Not detecting two side markers");
    center_tf_ = get_tf(center_marker_);
    if (center_tf_.header.frame_id.empty()){
      RCLCPP_ERROR(logger_, "Not detecting center marker, exit state");
      set_state(DockState::INVALID);
      return;
    }
    if (std::abs(center_tf_.pose.position.x) > transition_dis_with_tol_){
      RCLCPP_WARN(logger_, "Center too far, exit state");
      set_state(DockState::INVALID);
      return;
    }
    RCLCPP_WARN(logger_, "Center marker %.3f m away, transtion to last_mile", center_tf_.pose.position.x);
    remaining_dis_ = to_last_mile_dis_;
    set_state(DockState::LAST_MILE,"Last Mile.");
    return;
  }

  if (front_dock_){autodock_util::flip_base_frame(center_tf_);}
  double dis = center_tf_.pose.position.x;
  double offset = center_tf_.pose.position.y;
  if (dis > 0){
    stopRobot();
    remaining_dis_ = to_last_mile_dis_;
    set_state(DockState::LAST_MILE,"Last Mile.");
    return;
  }
  double ang_vel = autodock_util::sat_proportional_filter(-offset, 0,max_angular_vel_,offset_to_angular_vel_);
  
  // Collision check for autodock
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    set_state(DockState::INVALID);
    return;
  }
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = dir_*max_linear_vel_;
  cmd_vel->angular.z = ang_vel;
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = current_pose.pose.position.x;
  pose2d.y = current_pose.pose.position.y;
  pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(cmd_vel.get(), pose2d)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting Autodock");
    set_state(DockState::INVALID);
    return;
  }

  publish_cmd(dir_*max_linear_vel_,ang_vel);
  return;
}

void AutoDock::do_last_mile(){
  /**
   * @brief Function that runs in state machine LAST_MILE each cycle. Utilizes the center marker to move the robot closer to the charging station.
   * 
  */
  center_tf_ = get_tf(center_marker_);
  if (center_tf_.header.frame_id.empty()){
    RCLCPP_WARN(logger_, "Not detecting center marker");
    if (remaining_dis_ < max_last_mile_odom_){
      RCLCPP_WARN(logger_, "Move last mile distance %.3f m with odom", remaining_dis_);
      do_move(remaining_dis_,min_linear_vel_);
      return;
    }
    else{
      RCLCPP_ERROR(logger_, "exceeded max_last_mile_odom with "
                                 "remaining dis of %.3f, exit!",remaining_dis_);
      // set state -> idle
      return;
    }
  }
  
  if (front_dock_){autodock_util::flip_base_frame(center_tf_);}
  double dis = center_tf_.pose.position.x;
  double yaw = tf2::getYaw(center_tf_.pose.orientation);
  yaw -= 0.5*M_PI;
  remaining_dis_ = -dis - stop_distance_ - cam_offset_;
  RCLCPP_INFO(logger_, " Approaching Charger -> d: %.3f , yaw: %.3f, remaining dis: %.3f", dis,yaw,remaining_dis_);
  nav2_util::getCurrentPose(move_initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_); // Update move_initial_pose for use in odom_last_mile
  // set state -> idle
  if (remaining_dis_ <= 0.0){
    stopRobot();
    set_state(DockState::ACTIVATE_CHARGER,"STOP!! Reached destination");
    return;
  }
  double ang_vel = autodock_util::sat_proportional_filter(yaw,0.0,min_angular_vel_,0.5);
  publish_cmd(dir_*min_linear_vel_,ang_vel);
  return;
}

bool AutoDock::isCollisionFree(geometry_msgs::msg::Twist * cmd_vel, geometry_msgs::msg::Pose2D & pose2d){
  // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
  int cycle_count = 0;
  double sim_position_change;
  double sim_angle_change;
  // const double diff_dist = abs(command_x_) - distance;
  const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
  geometry_msgs::msg::Pose2D init_pose = pose2d;
  bool fetch_data = true;

  while (cycle_count < max_cycle_count) {
    sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
    sim_angle_change = cmd_vel->angular.z * (cycle_count / cycle_frequency_);
    pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
    pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
    pose2d.theta = init_pose.theta + sim_angle_change;
    cycle_count++;

    // if (diff_dist - abs(sim_position_change) <= 0.) {
    //   break;
    // } // TODO: Manage twist to goal. This section would break the loop when close to the goal but will stick to a simulate_ahead_time_ implementation only

    if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
      return false;
    }
    fetch_data = false;
  }
  return true;
}


}  // namespace nav2_autodock_behavior

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_autodock_behavior::AutoDock, nav2_core::Behavior)
