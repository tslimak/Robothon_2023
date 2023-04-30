/*
 * This file is part of the robothon2023 project.
 * https://gitlab.lrz.de/AM/robothon2023.git
 */

#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>

namespace Robothon2023::TaskPlanner {
//! Detects the task board pose.
/*!
 * Does multiply runs and averages the pose values.
 */
class TaskBoardDetector {
public:
    //! Return the estimated task board pose.
    /*!
     * Starts a ROS node with a client to get the end effector pose and a subscriber for the "object_pose_array" topic.
     * Averages out the pose values (\ref NUM_OF_SAMPLES).
     *
     * @return Estimated task board pose.
     */
    Eigen::Isometry3d getVisionIso3dPose();

private:
    //! Callback function for the subscriber to the "object_pose_array" topic.
    /*!
     * Stores the broadcasted poses in the containers, i.e. \ref m_detectedTaskBoardPositions and \ref
     * m_detectedTaskBoardOrienations.
     *
     * @param msg Broadcasted pose message.
     */
    void poseEstimationCallback(const geometry_msgs::PoseArray& msg);

    //! Number of samples used to average the task board poses provided by the vision.
    static constexpr int NUM_OF_SAMPLES = 30;

    //! Flag that indicates when the pose was detected.
    bool boardPoseDetected = false;

    //! Stores the number of already estimated pose samples.
    unsigned int currentSampleNumEstimation = 0;

    //! Container for the broadcasted positions.
    std::array<Eigen::Vector3d, NUM_OF_SAMPLES> detectedTaskBoardPositions;

    //! Container for the broadcasted orientations.
    std::array<Eigen::Quaterniond, NUM_OF_SAMPLES> detectedTaskBoardOrientations;
};
} // namespace Robothon::StateMachine
