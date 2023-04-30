/*
 * This file is part of the robothon2021 project.
 * https://gitlab.lrz.de/AM/robothon2021.git
 */

#include "task_planner/task_board_detector/TaskBoardDetector.hpp"
#include <dual_arm/GetPoses.h>
#include <dual_arm/utils/utility_functions.hpp>
#include <ros/node_handle.h>
#include <thread>

//! Return the estimated task board pose.
Eigen::Isometry3d Robothon2023::TaskPlanner::TaskBoardDetector::getVisionIso3dPose()
{
    // Initialize the ROS node and subscriber/client.
    ros::NodeHandle nh;
    // Instantiate the required service clients and action clients for the application.
    ros::ServiceClient getLinkEightFrameInBaseClient = nh.serviceClient<dual_arm::GetPoses>("/dual_arm/getFlangePoses");
    // Wait for the vision system to publish.
    ros::Rate rate(0.5);
    ros::Subscriber subPoseEstimation
        = nh.subscribe("object_pose_array", 1000, &TaskBoardDetector::poseEstimationCallback, this);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (!boardPoseDetected) {
        ROS_INFO("Waiting for Board Detection");
        ros::spinOnce();
        rate.sleep();
    }

    // Average the values obtained by the vision.
    Eigen::Vector3d avgTaskBoardPosition = Eigen::Vector3d::Zero();
    for (const auto& el : detectedTaskBoardPositions) {
        avgTaskBoardPosition += el;
    }
    Eigen::Quaterniond avgTaskBoardOrientation(0.0, 0.0, 0.0, 0.0);
    for (const auto& el : detectedTaskBoardOrientations) {
        avgTaskBoardOrientation.w() += el.w();
        avgTaskBoardOrientation.x() += el.x();
        avgTaskBoardOrientation.y() += el.y();
        avgTaskBoardOrientation.z() += el.z();
    }
    avgTaskBoardPosition /= static_cast<double>(NUM_OF_SAMPLES);
    avgTaskBoardOrientation.coeffs() /= static_cast<double>(NUM_OF_SAMPLES);
    // bROS_INFO_STREAM("The averaged task board position is: " << avgTaskBoardPosition);
    // ROS_INFO_STREAM("The averaged task board orientation is: " << avgTaskBoardOrientation.coeffs());

    // Call vision to get the task board pose.
    avgTaskBoardOrientation = detectedTaskBoardOrientations.at(10); // ToDo
    const Eigen::Isometry3d boardVisionRefInCamera
        = M2A::DualArm::posRot2mat(avgTaskBoardPosition, avgTaskBoardOrientation.normalized());

    // From calibration according to:
    // https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html.
    Eigen::Isometry3d cameraInPandaLink8 = M2A::DualArm::posRot2mat(
        Eigen::Vector3d(0.00178323, -0.074067, 0.0503125), Eigen::Quaterniond(0.913184, 0.128384, 0.0539182, 0.383021));

    // Get PandaLink8 frame w.r.t. the robot's base from the motion planner by a service call.
    dual_arm::GetPoses getLinkEightFrameTransform;
    getLinkEightFrameInBaseClient.call(getLinkEightFrameTransform);
    geometry_msgs::Pose linkEightInBaseTransform
        = getLinkEightFrameTransform.response.poses.at(M2A::DualArm::Robot::second);
    const Eigen::Isometry3d linkEightInBase
        = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second).inverse()
        * M2A::DualArm::ros2eigen(linkEightInBaseTransform);

    // Vision and motion planner have different board reference frames.
    const Eigen::Isometry3d boardPlannerRefInBoardVisionRef = M2A::DualArm::posRot2mat(
        Eigen::Vector3d(0.06625, 0.1165, 0.0), Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0).normalized());

    // Add up the transformations to get the board reference frame (of the motion planner) w.r.t. the robot's base.
    Eigen::Isometry3d boardPlannerRefInBase
        = linkEightInBase * cameraInPandaLink8 * boardVisionRefInCamera * boardPlannerRefInBoardVisionRef;

    return boardPlannerRefInBase;
}

//! Callback function for the subscriber to the "object_pose_array" topic.
void Robothon2023::TaskPlanner::TaskBoardDetector::poseEstimationCallback(const geometry_msgs::PoseArray& msg)
{
    if (msg.poses.empty()) {
        return;
    }
    if (boardPoseDetected) {
        return;
    }
    ROS_INFO_STREAM("TaskBoardDetector.cpp: Got "
        << msg.poses.at(0).position.x << " " << msg.poses.at(0).position.y << " " << msg.poses.at(0).position.z
        << " and orientation (w-x-y-z) \n"
        << msg.poses.at(0).orientation.w << " " << msg.poses.at(0).orientation.x << " " << msg.poses.at(0).orientation.y
        << " " << msg.poses.at(0).orientation.z << " ");
    Eigen::Vector3d currentPosition;
    currentPosition << msg.poses.at(0).position.x, msg.poses.at(0).position.y, msg.poses.at(0).position.z;
    detectedTaskBoardPositions.at(currentSampleNumEstimation) = currentPosition;
    detectedTaskBoardOrientations.at(currentSampleNumEstimation) = Eigen::Quaterniond(msg.poses.at(0).orientation.w,
        msg.poses.at(0).orientation.x, msg.poses.at(0).orientation.y, msg.poses.at(0).orientation.z);
    ++currentSampleNumEstimation;
    if (currentSampleNumEstimation == NUM_OF_SAMPLES) {
        boardPoseDetected = true;
    }
}
