/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include <math.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/macros/console_colors.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_interface_demo");
moveit::planning_interface::MoveGroupInterface *move_group_base_arm_ptr;
moveit::planning_interface::MoveGroupInterface *move_group_position_ptr;

bool keep_observing_arm = true;
bool m_ReadPose=false;

void TransformRPY(double phi, double theta, double epsi, Eigen::Matrix3d &R)
{
	R(0, 0) = cos(theta)*cos(epsi);
	R(0, 1) = cos(theta)*sin(epsi);
	R(0, 2) = -sin(theta);
	R(1, 0) = sin(phi)*sin(theta)*cos(epsi) - cos(phi)*sin(epsi);
	R(1, 1) = sin(phi)*sin(theta)*sin(epsi) + cos(phi)*cos(epsi);
	R(1, 2) = cos(theta)*sin(phi);
	R(2, 0) = cos(phi)*sin(theta)*cos(epsi) + sin(phi)*sin(epsi);
	R(2, 1) = cos(phi)*sin(theta)*sin(epsi) - sin(phi)*cos(epsi);
	R(2, 2) = cos(theta)*cos(phi);
}

void ObserveArmSeparate()
{
  RCLCPP_INFO(LOGGER, "========================================waly===============================");

  while(keep_observing_arm)
  {
    // geometry_msgs::msg::PoseStamped current_pose = move_group_base_arm_ptr->getCurrentPose();

    // Eigen::Quaterniond q1(
    //   current_pose.pose.orientation.w,
    //   current_pose.pose.orientation.x,
    //   current_pose.pose.orientation.y,
    //   current_pose.pose.orientation.z);
    // Eigen::Matrix3d r1(q1.normalized().toRotationMatrix());

    // RCLCPP_INFO(LOGGER, "thread Translation: %f %f %f",
    //   current_pose.pose.position.x,
    //   current_pose.pose.position.y,
    //   current_pose.pose.position.z);

    // RCLCPP_INFO(LOGGER, "thread Rotation: %f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f",
    //   r1(0,0),
    //   r1(0,1),
    //   r1(0,2),
    //   r1(1,0),
    //   r1(1,1),
    //   r1(1,2),
    //   r1(2,0),
    //   r1(2,1),
    //   r1(2,2));
    geometry_msgs::msg::PoseStamped current_pose = move_group_position_ptr->getCurrentPose();
    tf2::Quaternion q11(
          current_pose.pose.orientation.x,
          current_pose.pose.orientation.y,
          current_pose.pose.orientation.z,
          current_pose.pose.orientation.w);
    tf2::Matrix3x3 m11(q11);
    double roll11, pitch11, yaw11;
    m11.getRPY(roll11, pitch11, yaw11);

    RCLCPP_INFO(LOGGER, "grasp pose");
    RCLCPP_INFO(LOGGER, "x\t%f\ny\t%f\nz\t%f\nyaw\t%f\npitch\t%f\nroll\t%f\n", 
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    yaw11*180.0/M_PI,pitch11*180.0/M_PI,roll11*180.0/M_PI);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void poseMsgToEigen(const geometry_msgs::msg::Pose& msg, Eigen::Isometry3d& out)
{
  Eigen::Translation3d translation(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Quaterniond quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  if ((quaternion.x() == 0) && (quaternion.y() == 0) && (quaternion.z() == 0) && (quaternion.w() == 0))
  {
    RCLCPP_WARN(LOGGER, "Empty quaternion found in pose message. Setting to neutral orientation.");
    quaternion.setIdentity();
  }
  else
  {
    quaternion.normalize();
  }
  out = translation * quaternion;
}

void initializeMoveGroup(moveit::planning_interface::MoveGroupInterface& move_group)
{
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);
}

rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
geometry_msgs::msg::Transform tag_pose2;
geometry_msgs::msg::Pose goal_pose;

void CalcGraspOrientation(
  const Eigen::Matrix3d &object_r, 
  Eigen::Vector3d object_t,
  Eigen::Quaterniond &gripper_orientation,
  Eigen::Vector3d &gripper_position,
  Eigen::Quaterniond &gripper_pre_grasp_orientation,
  Eigen::Vector3d &gripper_pre_grasp_position)
{
  // double cam_seeing_imu_roll = -1.556417;
  // double cam_seeing_imu_pitch = -0.003150;
  // double cam_seeing_imu_yaw = 0.021740;
  // Eigen::Matrix3d cam_seeing_imu_rot;
  // Eigen::Matrix3d imu_seeing_cam_rot;
  // TransformRPY(cam_seeing_imu_roll, cam_seeing_imu_pitch, cam_seeing_imu_yaw, cam_seeing_imu_rot);
  // imu_seeing_cam_rot = cam_seeing_imu_rot.transpose();
  // return imu_seeing_cam_rot;
  Eigen::Vector3d object_t_original = object_t;
  object_t(2)= 0.0;

  geometry_msgs::msg::PoseStamped current_pose = move_group_position_ptr->getCurrentPose();

  Eigen::Vector3d t_odom_seeing_base_link;
  t_odom_seeing_base_link(0) = current_pose.pose.position.x;
  t_odom_seeing_base_link(1) = current_pose.pose.position.y;
  t_odom_seeing_base_link(2) = 0.0;

  Eigen::Quaterniond q(
    current_pose.pose.orientation.w,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z);

  Eigen::Matrix3d r_base_link_seeing_odom(q.normalized().toRotationMatrix());

  Eigen::Vector3d t_base_link_seeing_link0_as_point;
  t_base_link_seeing_link0_as_point(0) = 0.275041;
  t_base_link_seeing_link0_as_point(1) = -0.100092;
  t_base_link_seeing_link0_as_point(2) = 0.0;
  
  Eigen::Vector3d odom_seeing_link0 = r_base_link_seeing_odom * t_base_link_seeing_link0_as_point + t_odom_seeing_base_link;

  Eigen::Vector3d v = object_t - odom_seeing_link0;
  RCLCPP_INFO(LOGGER, "\nv.x\t%f\nv.y\t%f\nv.z\t%f", v(0), v(1), v(2));
  
  double yaw = atan2(v(1),v(0))+M_PI;
  double pitch = -150.0*M_PI/180.0;
  double roll = 0.0;
  Eigen::Matrix3d odom_seeing_gripper_grasp;
  TransformRPY(roll, pitch, yaw, odom_seeing_gripper_grasp);
  Eigen::Matrix3d gripper_grasp_seeing_odom = odom_seeing_gripper_grasp.transpose();
  Eigen::Vector3d gripper_z;
  gripper_z(0) = odom_seeing_gripper_grasp(2,0);
  gripper_z(1) = odom_seeing_gripper_grasp(2,1);
  gripper_z(2) = odom_seeing_gripper_grasp(2,2);
  
  gripper_position = object_t_original - gripper_z*0.10;//0.15
  gripper_orientation = gripper_grasp_seeing_odom;
  gripper_position(2) += 0.03;
  
  tf2::Quaternion q11(
          gripper_orientation.x(),
          gripper_orientation.y(),
          gripper_orientation.z(),
          gripper_orientation.w());
  tf2::Matrix3x3 m11(q11);
  double roll11, pitch11, yaw11;
  m11.getRPY(roll11, pitch11, yaw11);

  RCLCPP_INFO(LOGGER, "grasp pose");
  RCLCPP_INFO(LOGGER, "x\t%f\ny\t%f\nz\t%f\nyaw\t%f\npitch\t%f\nroll\t%f\n", 
  gripper_position(0),
  gripper_position(1),
  gripper_position(2),
  yaw11*180.0/M_PI,pitch11*180.0/M_PI,roll11*180.0/M_PI);

  Eigen::Vector3d gripper_z_horizontal = gripper_z;
  gripper_z_horizontal(2) = 0.0;
  gripper_z_horizontal.normalize();
  RCLCPP_INFO(LOGGER, "grasp gripper_z_horizontal");
  RCLCPP_INFO(LOGGER, "x\t%f\ny\t%f\nz\t%f\n", 
  gripper_z_horizontal(0),
  gripper_z_horizontal(1),
  gripper_z_horizontal(2));

  gripper_pre_grasp_position = gripper_position - gripper_z_horizontal * 0.07;
  RCLCPP_INFO(LOGGER, "pre grasp");
  RCLCPP_INFO(LOGGER, "x\t%f\ny\t%f\nz\t%f\n", 
  gripper_pre_grasp_position(0),
  gripper_pre_grasp_position(1),
  gripper_pre_grasp_position(2));

  gripper_pre_grasp_orientation = gripper_orientation;
}

void setupPlanningScene(rclcpp::Node::SharedPtr node_, moveit::planning_interface::PlanningSceneInterface &psi, moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  Eigen::Matrix3d cam_r;
  Eigen::Vector3d cam_t;
  cam_t(0) = 0.0645;
  cam_t(1) = 0.0;
  cam_t(2) = 0.0685;
  
  cam_r(0,0)=0.0;   cam_r(0,1)=-1.0; cam_r(0,2)=0.0;
  cam_r(1,0)=1.0;   cam_r(1,1)=0.0;  cam_r(1,2)=0.0;
  cam_r(2,0)=0.0;   cam_r(2,1)=0.0;  cam_r(2,2)=1.0;

  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // q.setRPY(roll,pitch,yaw);
  Eigen::Quaterniond tag_pose_q(
    tag_pose2.rotation.w,
    tag_pose2.rotation.x,
    tag_pose2.rotation.y,
    tag_pose2.rotation.z);

  Eigen::Matrix3d tag_pose_r(tag_pose_q.normalized().toRotationMatrix());
  Eigen::Vector3d tag_pose_t;
  tag_pose_t(0) = tag_pose2.translation.x;
  tag_pose_t(1) = tag_pose2.translation.y;
  tag_pose_t(2) = tag_pose2.translation.z;

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  Eigen::Quaterniond hand_pose_q(
    current_pose.pose.orientation.w,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z);

  Eigen::Matrix3d hand_pose_r(hand_pose_q.normalized().toRotationMatrix());
  Eigen::Vector3d hand_pose_t;
  hand_pose_t(0) = current_pose.pose.position.x;
  hand_pose_t(1) = current_pose.pose.position.y;
  hand_pose_t(2) = current_pose.pose.position.z;

  RCLCPP_INFO(LOGGER, "========================================waly===============================");

  RCLCPP_INFO(LOGGER, "waly Current pose: %f %f %f %f %f %f %f",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w);


  Eigen::Matrix3d R = hand_pose_r * cam_r * tag_pose_r;
  Eigen::Vector3d T = hand_pose_r * cam_r * tag_pose_t + hand_pose_r * cam_t + hand_pose_t;

  Eigen::Quaterniond Q(R);

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "odom";
  object.primitives.resize(1);
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  // object.primitives[0].dimensions = { 0.1, 0.02 };
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { 0.1, 0.1, 0.1 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = T(0);
  pose.position.y = T(1);
  pose.position.z = T(2);
  pose.orientation.w = Q.w();
  pose.orientation.x = Q.x();
  pose.orientation.y = Q.y();
  pose.orientation.z = Q.z();

  tf2::Quaternion q11(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w);
  tf2::Matrix3x3 m11(q11);
  double roll11, pitch11, yaw11;
  m11.getRPY(roll11, pitch11, yaw11);

  RCLCPP_INFO(LOGGER, "x\t%f\ny\t%f\nz\t%f\nyaw\t%f\npitch\t%f\nroll\t%f\n", 
  pose.position.x,
  pose.position.y,
  pose.position.z,
  yaw11,pitch11,roll11);

  // pose.orientation.w = 1.0;
  // pose.orientation.x = 0.0;
  // pose.orientation.y = 0.0;
  // pose.orientation.z = 0.0;
  object.pose = pose;
  goal_pose = pose;
  // geometry_msgs::msg::Pose pose;
  // pose.position.x = 0.0;
  // pose.position.y = 0.4;
  // pose.position.z = 0.4;
  // // pose.position.y = 0.4;
  // // pose.position.z = 0.05;
  // pose.orientation.w = 1.0;
  // object.pose = pose;

  // psi.applyCollisionObject(object);
}

void updatePlanningScene(rclcpp::Node::SharedPtr node_, moveit::planning_interface::PlanningSceneInterface &psi, moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  Eigen::Matrix3d cam_r;
  Eigen::Vector3d cam_t;
  cam_t(0) = 0.0645;
  cam_t(1) = 0.0;
  cam_t(2) = 0.0685;
  
  cam_r(0,0)=0.0;   cam_r(0,1)=-1.0; cam_r(0,2)=0.0;
  cam_r(1,0)=1.0;   cam_r(1,1)=0.0;  cam_r(1,2)=0.0;
  cam_r(2,0)=0.0;   cam_r(2,1)=0.0;  cam_r(2,2)=1.0;

  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // q.setRPY(roll,pitch,yaw);
  Eigen::Quaterniond tag_pose_q(
    tag_pose2.rotation.w,
    tag_pose2.rotation.x,
    tag_pose2.rotation.y,
    tag_pose2.rotation.z);

  Eigen::Matrix3d tag_pose_r(tag_pose_q.normalized().toRotationMatrix());
  Eigen::Vector3d tag_pose_t;
  tag_pose_t(0) = tag_pose2.translation.x;
  tag_pose_t(1) = tag_pose2.translation.y;
  tag_pose_t(2) = tag_pose2.translation.z;

  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  Eigen::Quaterniond hand_pose_q(
    current_pose.pose.orientation.w,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z);

  Eigen::Matrix3d hand_pose_r(hand_pose_q.normalized().toRotationMatrix());
  Eigen::Vector3d hand_pose_t;
  hand_pose_t(0) = current_pose.pose.position.x;
  hand_pose_t(1) = current_pose.pose.position.y;
  hand_pose_t(2) = current_pose.pose.position.z;

  RCLCPP_INFO(LOGGER, "========================================waly===============================");

  RCLCPP_INFO(LOGGER, "waly Current pose: %f %f %f %f %f %f %f",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w);


  Eigen::Matrix3d R = hand_pose_r * cam_r * tag_pose_r;
  Eigen::Vector3d T = hand_pose_r * cam_r * tag_pose_t + hand_pose_r * cam_t + hand_pose_t;

  Eigen::Quaterniond Q(R);

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "odom";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  // object.primitives[0].dimensions = { 0.04, 0.04, 0.04 };

  geometry_msgs::msg::Pose pose;
  pose.position.x = 2.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = Q.w();
  pose.orientation.x = Q.x();
  pose.orientation.y = Q.y();
  pose.orientation.z = Q.z();
  // pose.orientation.w = 1.0;
  // pose.orientation.x = 0.0;
  // pose.orientation.y = 0.0;
  // pose.orientation.z = 0.0;
  object.pose = pose;
  goal_pose = pose;
  // geometry_msgs::msg::Pose pose;
  // pose.position.x = 0.0;
  // pose.position.y = 0.4;
  // pose.position.z = 0.4;
  // // pose.position.y = 0.4;
  // // pose.position.z = 0.05;
  // pose.orientation.w = 1.0;
  // object.pose = pose;

  // psi.applyCollisionObject(object);
}


void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  // RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg->header.frame_id.c_str());
  tag_pose2.rotation.x = msg->transform.rotation.x;
  tag_pose2.rotation.y = msg->transform.rotation.y;
  tag_pose2.rotation.z = msg->transform.rotation.z;
  tag_pose2.rotation.w = msg->transform.rotation.w;
  tag_pose2.translation.x = msg->transform.translation.x;
  tag_pose2.translation.y = msg->transform.translation.y;
  tag_pose2.translation.z = msg->transform.translation.z;
  m_ReadPose=true;
}

void InitPickAndPlace(rclcpp::Node::SharedPtr node_, moveit::planning_interface::PlanningSceneInterface &psi, moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  subscription_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>("pose2", 10, topic_callback);
  while(!m_ReadPose)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(1000000000));
  }

  setupPlanningScene(node_, psi, move_group_interface);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_demo", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_gripper(move_group_node, "hand");
  moveit::planning_interface::MoveGroupInterface move_group_base_arm(move_group_node, "mobile_base_arm");
  moveit::planning_interface::MoveGroupInterface move_group_position(move_group_node, "position");
  moveit::planning_interface::MoveGroupInterface move_group_robot_arm(move_group_node, "robot_arm");

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  namespace rvt = rviz_visual_tools;
  rviz_visual_tools::RvizVisualTools visual_tools("robot_link0", "move_group_tutorial", move_group_node);
  /* moveit_visual_tools::MoveItVisualTools visual_tools("robot_link0"); */
  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_base_arm.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_base_arm.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group_base_arm.getJointModelGroupNames().begin(), move_group_base_arm.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Setting Initial Planning Parameters
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  initializeMoveGroup(move_group_base_arm);
  initializeMoveGroup(move_group_gripper);
  initializeMoveGroup(move_group_robot_arm);
  
  move_group_base_arm_ptr = &move_group_base_arm;
  move_group_position_ptr = &move_group_position;
  geometry_msgs::msg::Pose target_pose;
  Eigen::Isometry3d approx_target = Eigen::Isometry3d::Identity();
  bool success;
  geometry_msgs::msg::PoseStamped current_pose = move_group_base_arm.getCurrentPose();

  // visual_tools.prompt("Press 'next' to test");
  // // move_group_gripper.setStartStateToCurrentState();
  // // move_group_gripper.setNamedTarget("open");
  // // success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // // RCLCPP_INFO(LOGGER, "Opening gripper execution %s", success ? "" : "FAILED");

  // move_group_robot_arm.setStartStateToCurrentState();
  // move_group_robot_arm.setNamedTarget("look_forward");
  // success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Move to test %s", success ? "" : "FAILED");

  // move_group_robot_arm.setStartStateToCurrentState();
  // // std::vector<double> joint_values;
  // // joint_values.push_back(0.0);
  // // joint_values.push_back(-0.331612558);
  // // joint_values.push_back(1.623156204);
  // // joint_values.push_back(0.0);
  // // joint_values.push_back(0.907571211);
  // // joint_values.push_back(0.0);
  
  // // move_group_robot_arm.setJointValueTarget(joint_values);
  // move_group_robot_arm.setNamedTarget("look_forward");
  // success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Move to test %s", success ? "" : "FAILED");

  // auto group = t.getRobotModel()->getJointModelGroup("panda_arm");
  // scene = std::make_shared<PlanningScene>(t.getRobotModel());
  // scene->getCurrentStateNonConst().setToDefaultValues();
  // scene->getCurrentStateNonConst().setToDefaultValues(group, "extended");

  // s.setToDefaultValues(s.getJointModelGroup("bar"), "foo");
  // moveit::core::RobotState& current_state{ planning_scene->getCurrentStateNonConst() };
  // current_state.setToDefaultValues(current_state.getJointModelGroup("panda_arm"), "home");
  // current_state.update();

  // // Part 6: Move the robot towards the other table and make a turn
  // visual_tools.prompt("Press 'next' to move the base back");
  // move_group_position.setStartStateToCurrentState();
  // move_group_position.setJointValueTarget("position", {-1.062839, 0.0, 3.004139789});
  // success = (move_group_position.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Move the robot execution %s", success ? "" : "FAILED");

  // visual_tools.prompt("Press 'next' to move the arm to place1");
  // move_group_robot_arm.setStartStateToCurrentState();
  // move_group_robot_arm.setNamedTarget("place1");
  // success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "place the object %s", success ? "" : "FAILED");

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' to look forward");
  move_group_robot_arm.setStartStateToCurrentState();
  move_group_robot_arm.setNamedTarget("look_forward");
  success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "look forward %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' to see apriltag");
  // std::thread th1(ObserveArmSeparate);
  // th1.detach();
  InitPickAndPlace(move_group_node, planning_scene_interface, move_group_robot_arm);
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  // Part 0: Turn your head to the target

  //test update planning scene
  // visual_tools.prompt("Press 'next' to test");
  // setupPlanningScene(move_group_node, planning_scene_interface, move_group_base_arm);

  // // Part 1: Move Lift to height
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // move_group_base_arm.setStartStateToCurrentState();

  // Print the current pose of the end effector
  // RCLCPP_INFO(LOGGER, "Current pose: %f %f %f %f %f %f %f",
  //   current_pose.pose.position.x,
  //   current_pose.pose.position.y,
  //   current_pose.pose.position.z,
  //   current_pose.pose.orientation.x,
  //   current_pose.pose.orientation.y,
  //   current_pose.pose.orientation.z,
  //   current_pose.pose.orientation.w);

  // target_pose.position.x = current_pose.pose.position.x;
  // target_pose.position.y = current_pose.pose.position.y+0.01;
  // target_pose.position.z = current_pose.pose.position.z-0.01;
  // target_pose.orientation.x = current_pose.pose.orientation.x;
  // target_pose.orientation.y = current_pose.pose.orientation.y;
  // target_pose.orientation.z = current_pose.pose.orientation.z;
  // target_pose.orientation.w = current_pose.pose.orientation.w;
  // poseMsgToEigen(target_pose, approx_target);

  // move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are planning and asking move_group
  // // to actually move the robot. 
  // // If we wanted to plan only we would use move.group.plan(plan) instead
  // success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Moving arm to height execution %s", success ? "" : "FAILED");


  // Part 2: Open Gripper
  visual_tools.prompt("Press 'next' to open gripper");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("open");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Opening gripper execution %s", success ? "" : "FAILED");
  
  // Part 3: Move to grabbing pose
  visual_tools.prompt("Press 'next' to move gripper to the pre grasp pose");

  Eigen::Quaterniond tag_pose_q(
    goal_pose.orientation.w,
    goal_pose.orientation.x,
    goal_pose.orientation.y,
    goal_pose.orientation.z);

  Eigen::Matrix3d tag_pose_r(tag_pose_q.normalized().toRotationMatrix());
  Eigen::Vector3d tag_pose_t;
  tag_pose_t(0) = goal_pose.position.x;
  tag_pose_t(1) = goal_pose.position.y;
  tag_pose_t(2) = goal_pose.position.z;

  Eigen::Quaterniond goal_q, pre_goal_q;
  Eigen::Vector3d goal_t, pre_goal_t;

  CalcGraspOrientation(tag_pose_r,tag_pose_t,goal_q, goal_t, pre_goal_q, pre_goal_t);

  target_pose.position.x = goal_t(0);
  target_pose.position.y = goal_t(1);
  target_pose.position.z = goal_t(2);
  target_pose.orientation.x = goal_q.x();
  target_pose.orientation.y = goal_q.y();
  target_pose.orientation.z = goal_q.z();
  target_pose.orientation.w = goal_q.w();
  poseMsgToEigen(target_pose, approx_target);
  
  move_group_robot_arm.setStartStateToCurrentState();
  move_group_robot_arm.setApproximateJointValueTarget(approx_target, move_group_robot_arm.getEndEffectorLink());
  success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Move to pre grabbing pose execution %s", success ? "" : "FAILED");


  // // Part 3.1: Move to grabbing pose
  // visual_tools.prompt("Press 'next' to move gripper to the object");
  // move_group_base_arm.setStartStateToCurrentState();

  // target_pose.position.x = goal_t(0);
  // target_pose.position.y = goal_t(1);
  // target_pose.position.z = goal_t(2);
  // target_pose.orientation.x = goal_q.x();
  // target_pose.orientation.y = goal_q.y();
  // target_pose.orientation.z = goal_q.z();
  // target_pose.orientation.w = goal_q.w();
  // poseMsgToEigen(target_pose, approx_target);
  
  // move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());
  // success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Move to grabbing pose execution %s", success ? "" : "FAILED");

  // Part 4: Close Gripper and grab the object
  visual_tools.prompt("Press 'next' to close gripper");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("closed");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Closing gripper execution %s", success ? "" : "FAILED");

  // Part 5: Lift the object slightly
  visual_tools.prompt("Press 'next' lift the object");

  // move_group_base_arm.setStartStateToCurrentState();
  // target_pose.position.z += 0.15;
  // target_pose.position.y += 0.5;
  // poseMsgToEigen(target_pose, approx_target);
  
  // move_group_base_arm.setApproximateJointValueTarget(approx_target, move_group_base_arm.getEndEffectorLink());
  // success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Lift the object execution %s", success ? "" : "FAILED");

  move_group_robot_arm.setStartStateToCurrentState();
  move_group_robot_arm.setNamedTarget("look_forward");
  success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Lift the object %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' look_forward_0");
  move_group_robot_arm.setStartStateToCurrentState();
  move_group_robot_arm.setNamedTarget("look_forward_0");
  success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "look_forward_0 %s", success ? "" : "FAILED");

  // Part 6: Move the robot towards the other table and make a turn
  visual_tools.prompt("Press 'next' to move the base back");
  move_group_position.setStartStateToCurrentState();
  move_group_position.setJointValueTarget("position", {-1.062839, 0.0, 3.004139789});
  success = (move_group_position.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Move the robot execution %s", success ? "" : "FAILED");

  // visual_tools.prompt("Press 'next' to rotate the base");
  // move_group_base_arm.setStartStateToCurrentState();
  // move_group_base_arm.setJointValueTarget("position", {-0.50, 0, -1.570796327});
  // success = (move_group_base_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // RCLCPP_INFO(LOGGER, "Turn the robot execution %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' to move the arm to place1");
  move_group_robot_arm.setStartStateToCurrentState();
  move_group_robot_arm.setNamedTarget("place1");
  success = (move_group_robot_arm.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "place the object %s", success ? "" : "FAILED");

  // Part 7: Open gripper and drop the object
  visual_tools.prompt("Press 'next' to open the gripper");
  move_group_gripper.setStartStateToCurrentState();
  move_group_gripper.setNamedTarget("open");
  success = (move_group_gripper.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Opening gripper execution %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
  rclcpp::shutdown();
  return 0;
}
