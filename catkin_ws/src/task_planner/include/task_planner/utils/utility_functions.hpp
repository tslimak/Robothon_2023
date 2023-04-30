/*
 * This file is part of the robothon2023 project.
 * https://gitlab.lrz.de/AM/robothon2023
 */

#pragma once

#include <Eigen/Geometry>
#include <array>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <dual_arm/robot/Robot.hpp>
#include <dual_arm/ForceAction.h>
#include <dual_arm/utils/PlannerConfig.hpp>
#include <geometry_msgs/Pose.h>
#include <optional>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <optional>

namespace Robothon2023::TaskPlanner {
//! Home both robots.
/*!
 * @param client Client that calls the server.
 */
void home(ros::ServiceClient& client);

//! Move the robots to given poses w.r.t. world frame.
/*!
 * @param poses Goal poses of the TCPs w.r.t. the world frame. Might be empty if only one robot should move.
 * @param client Client that calls the motion server.
 */
void move(const std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS>& poses, ros::ServiceClient& client,
    bool useErrorCompensation = true);

//! Move the robots to given configurations.
/*!
 * @param configs Goal configurations w.r.t. the world frame. Might be empty if
 * only one robot should move.
 * @param client Client that calls the motion server.
 */
void move(
    const std::array<std::optional<Eigen::Matrix<double, 7, 1>>, NUM_OF_ROBOTS>& configs, ros::ServiceClient& client);

//! Cooperative motion of the robots to given poses w.r.t. world frame.
/*!
 * @param object ID of the object to be moved in a cooperative way.
 * @param pose Goal pose of the cooperative object w.r.t. the world frame.
 * @param client Client that calls the motion server.
 */
void coopMove(const std::string& object, const std::vector<geometry_msgs::Pose>& waypoints, ros::ServiceClient& client);

//! Move one end effector along a path.
/*!
 * @param robot Robot to move.
 * @param poses Path waypoints w.r.t. world frame.
 * @param client Client that calls the motion server.
 */
void move(const M2A::DualArm::Robot& robot, const std::vector<Eigen::Isometry3d>& poses, ros::ServiceClient& client);

//! Move one end effector along a path.
/*!
 * @param robot Robot to move.
 * @param poses Path waypoints w.r.t. world frame.
 * @param client Client that calls the motion server.
 */
void move(const M2A::DualArm::Robot& robot, const std::vector<Eigen::Matrix<double,7,1>>& configs, ros::ServiceClient& client);

//! Add an object to the environment.
/*!
 * @param object Object name that must be the same as the information on the ROS Parameter Server.
 * @param client Client that calls the server to add the object.
 * @param desiredPoseInWorld If no pose is passed, the default is loaded from
 * the Ros Parameter Server.
 */
void addEnvironmentObject(
    const std::string& object, ros::ServiceClient& client, std::optional<Eigen::Isometry3d> desiredPoseInWorld = {});

void deleteTaskBoard(ros::ServiceClient& client);

//! Grasp.
/*!
 * @param graspingWidth Grasping widths.
 * @param client Client that calls the grasp server.
 */
void grasp(const std::array<std::optional<double>, NUM_OF_ROBOTS>& graspingWidth, ros::ServiceClient& client, double force);

//! Open gripper of a specific robot.
/*!
 * @param robot Robot that should open its gripper.
 * @param width Opening width.
 * @param keepObjectInWorld Keep a released object in the world?
 * @param client Client that calls the moveGripper server.
 */
void moveGripper(const M2A::DualArm::Robot& robot, float width, bool keepObjectInWorld, ros::ServiceClient& client);

//! Localizes the taskboard and returns its pose w.r.t. the robot base.
/*!
 * @param client Client to perform a coordinated move in joint space.
 * @return Cartesian pose of the taskboard w.r.t. the robot base.
 */
std::optional<Eigen::Isometry3d> localizeTaskBoard(ros::ServiceClient& client);

//! Get a transformation matrix that implements an offset in z direction.
/*!
 * @param offset Offset in z in [m].
 * @return Homogeneous transformation matrix.
 */
Eigen::Isometry3d zOffset(double offset);

//! Get the pose for a task w.r.t. the planner reference frame in the corner of the board.
/*!
 * Reads the task pose from the ROS parameter server.
 *
 * @param task Task name.
 * @return Pose of the task w.r.t. the planner reference frame.
 */
Eigen::Isometry3d taskPose(const std::string& task);

//! Calls the server that applies a Cartesian force for a given task.
/*!
 * @param robot Robot that has to apply the force.
 * @param task Task that defines what force (and whether with wiggling) to apply.
 * @param client Client that calls the server that applies the force.
 */
void force(const M2A::DualArm::Robot& robot, std::string task, ros::ServiceClient& client);

void forceAction(const M2A::DualArm::Robot& robot, std::string task, actionlib::SimpleActionClient<dual_arm::ForceAction>& client);


//! Compute planar circular path waypoints to open the lid.
/*!
 * @param start Starting waypoint of the path.
 * @param radius Radius of the path in [m].
 * @param angle Angle of the circular segment in degree.
 * @return Circular path waypoints.
 */
std::vector<Eigen::Isometry3d> circleWaypoints(const Eigen::Isometry3d& start, double radius, double angle);

} // namespace Robothon2023::TaskPlanner
