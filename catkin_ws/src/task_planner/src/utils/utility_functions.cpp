/*
 * This file is part of the robothon2023 project.
 * https://gitlab.lrz.de/AM/robothon2023
 */

#include "task_planner/utils/utility_functions.hpp"
#include "task_planner/task_board_detector/TaskBoardDetector.hpp"
#include <dual_arm/AddEnvironment.h>
#include <dual_arm/CoopMove.h>
#include <dual_arm/CoordMoveJS.h>
#include <dual_arm/CoordMoveTS.h>
#include <dual_arm/Force.h>
#include <dual_arm/Grasp.h>
#include <dual_arm/MoveGripper.h>
#include <dual_arm/utils/utility_functions.hpp>
#include <math.h>
#include <thread>

//! Home both robots.
void Robothon2023::TaskPlanner::home(ros::ServiceClient& client)
{
    dual_arm::CoordMoveJS coordMoveJSCall;
    sensor_msgs::JointState coordMoveGoalConfig1;
    coordMoveGoalConfig1.position = M2A::DualArm::eigen2stdVector<7>(M2A::DualArm::PlannerConfig::get().m_home.at(0));
    sensor_msgs::JointState coordMoveGoalConfig2;
    coordMoveGoalConfig2.position = M2A::DualArm::eigen2stdVector<7>(M2A::DualArm::PlannerConfig::get().m_home.at(1));
    coordMoveJSCall.request.waypoints1.emplace_back(coordMoveGoalConfig1);
    coordMoveJSCall.request.waypoints2.emplace_back(coordMoveGoalConfig2);
    if (!client.call(coordMoveJSCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
    }
}

//! Move the robots to given poses w.r.t. world frame.
void Robothon2023::TaskPlanner::move(const std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS>& poses,
    ros::ServiceClient& client, bool useErrorCompensation)
{
    // Define the single motion in task space.
    dual_arm::CoordMoveTS moveCall;
    moveCall.request.waypoints1.clear();
    moveCall.request.waypoints2.clear();
    moveCall.request.useErrorCompensation = useErrorCompensation;
    if (poses.at(0).has_value()) {
        moveCall.request.waypoints1.emplace_back(M2A::DualArm::eigen2ros(poses.at(0).value()));
    }
    if (poses.at(1).has_value()) {
        moveCall.request.waypoints2.emplace_back(M2A::DualArm::eigen2ros(poses.at(1).value()));
    }
    if (!client.call(moveCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
        }
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
        }
    }
}

//! Move the robots to given configurations.
void Robothon2023::TaskPlanner::move(
    const std::array<std::optional<Eigen::Matrix<double, 7, 1>>, NUM_OF_ROBOTS>& configs, ros::ServiceClient& client)
{
    // Define the single motion in task space.
    dual_arm::CoordMoveJS moveCall;
    moveCall.request.waypoints1.clear();
    moveCall.request.waypoints2.clear();
    if (configs.at(0).has_value()) {
        sensor_msgs::JointState message;
        message.position = M2A::DualArm::eigen2stdVector<7>(configs.at(0).value());
        moveCall.request.waypoints1.emplace_back(message);
    }
    if (configs.at(1).has_value()) {
        sensor_msgs::JointState message;
        message.position = M2A::DualArm::eigen2stdVector<7>(configs.at(1).value());
        moveCall.request.waypoints2.emplace_back(message);
    }
    if (!client.call(moveCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
        }
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
        }
    }
}

//! Cooperative motion of the robots to given poses w.r.t. world frame.
void Robothon2023::TaskPlanner::coopMove(
    const std::string& object, const std::vector<geometry_msgs::Pose>& waypoints, ros::ServiceClient& client)
{
    dual_arm::CoopMove moveCall;
    moveCall.request.object.data = object;
    moveCall.request.waypoints = waypoints;
    if (!client.call(moveCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoopMove!");
    }
}

//! Move one end effector along a path.
void Robothon2023::TaskPlanner::move(
    const M2A::DualArm::Robot& robot, const std::vector<Eigen::Isometry3d>& poses, ros::ServiceClient& client)
{
    // Convert poses to ROS format.
    std::vector<geometry_msgs::Pose> rosPoses(0);
    for (const auto& transform : poses) {
        rosPoses.push_back(M2A::DualArm::eigen2ros(transform));
    }
    // Define the single motion in task space.
    dual_arm::CoordMoveTS moveCall;
    moveCall.request.waypoints1.clear();
    moveCall.request.waypoints2.clear();
    moveCall.request.useErrorCompensation = false;
    if (robot == M2A::DualArm::Robot::first) {
        moveCall.request.waypoints1 = rosPoses;
    }
    if (robot == M2A::DualArm::Robot::second) {
        moveCall.request.waypoints2 = rosPoses;
    }
    if (!client.call(moveCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
        }
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveTS!");
        }
    }
}

void Robothon2023::TaskPlanner::move(const M2A::DualArm::Robot& robot, const std::vector<Eigen::Matrix<double,7,1>>& configs, ros::ServiceClient& client){


    // Convert poses to ROS format.
    std::vector<sensor_msgs::JointState> rosConfigs(0);
    for (const auto& config : configs) {
        sensor_msgs::JointState message;
        message.position = M2A::DualArm::eigen2stdVector<7>(config);
        rosConfigs.push_back(message);
    }
    // Define the single motion in task space.
    dual_arm::CoordMoveJS moveCall;
    moveCall.request.waypoints1.clear();
    moveCall.request.waypoints2.clear();

    if (robot==M2A::DualArm::Robot::first) {
        moveCall.request.waypoints1 = rosConfigs;
    } else{
        moveCall.request.waypoints2 = rosConfigs;

    }
    if (!client.call(moveCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
        }
    }
    if (!moveCall.response.success) {
        if (!client.call(moveCall)) {
            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
        }
    }
}



//! Add an object to the environment.
void Robothon2023::TaskPlanner::addEnvironmentObject(
    const std::string& object, ros::ServiceClient& client, std::optional<Eigen::Isometry3d> desiredPoseInWorld)
{
    dual_arm::AddEnvironment call;
    call.request.name.data = object;
    if (!desiredPoseInWorld.has_value()) {
        // We get the desired pose from the ROS Parameter Server, but this could also be customized.
        XmlRpc::XmlRpcValue objectInformation;
        ros::param::get("/dual_arm/" + object, objectInformation);
        auto position = objectInformation["defaultTranslationInParent"];
        auto orientation = objectInformation["defaultOrientationInParent"];
        Eigen::Isometry3d objectPose
            = M2A::DualArm::posRot2mat(Eigen::Vector3d{ static_cast<double>(position[0]),
                                           static_cast<double>(position[1]), static_cast<double>(position[2]) },
                Eigen::Quaterniond{ static_cast<double>(orientation[0]), static_cast<double>(orientation[1]),
                    static_cast<double>(orientation[2]), static_cast<double>(orientation[3]) });
        call.request.pose = M2A::DualArm::eigen2ros(objectPose);
        call.request.parent.data = static_cast<std::string>(objectInformation["parent"]);
    } else {
        call.request.pose = M2A::DualArm::eigen2ros(desiredPoseInWorld.value());
        call.request.parent.data = "world";
    }
    if (!client.call(call)) {
        ROS_ERROR_STREAM("utility_functions.cpp: Failed to add object " << object << " to the environment.");
    }
}


void Robothon2023::TaskPlanner::deleteTaskBoard(ros::ServiceClient& client){
    dual_arm::AddEnvironment call;
    call.request.name.data = "task_board";

    XmlRpc::XmlRpcValue objectInformation;
    ros::param::get("/dual_arm/task_board", objectInformation);
    auto position = objectInformation["defaultTranslationInParent"];
    auto orientation = objectInformation["defaultOrientationInParent"];
    Eigen::Isometry3d objectPose
            = M2A::DualArm::posRot2mat(Eigen::Vector3d{ static_cast<double>(position[0]),
                                                        static_cast<double>(position[1]), 20.0},
                                       Eigen::Quaterniond{ static_cast<double>(orientation[0]), static_cast<double>(orientation[1]),
                                                           static_cast<double>(orientation[2]), static_cast<double>(orientation[3]) });
    call.request.pose = M2A::DualArm::eigen2ros(objectPose);
    call.request.parent.data = static_cast<std::string>(objectInformation["parent"]);
    if (!client.call(call)) {
        ROS_ERROR_STREAM("utility_functions.cpp: Failed to remove task_board ");
    }
}

//! Grasp an object.
void Robothon2023::TaskPlanner::grasp(
    const std::array<std::optional<double>, NUM_OF_ROBOTS>& graspingWidth, ros::ServiceClient& client, double force)
{
    dual_arm::Grasp graspCall;
    graspCall.request.epsilon = 0.1; // Currently, we do not properly check for grasp success.
    graspCall.request.width1.clear();
    graspCall.request.width2.clear();
    if (graspingWidth.at(0).has_value()) {
        graspCall.request.width1.push_back(graspingWidth.at(0).value());
    }
    if (graspingWidth.at(1).has_value()) {
        graspCall.request.width2.push_back(graspingWidth.at(1).value());
    }
    graspCall.request.force = force;
    if (!client.call(graspCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::Grasp!");
    }
}

//! Open gripper of a specific robot.
void Robothon2023::TaskPlanner::moveGripper(
    const M2A::DualArm::Robot& robot, float width, bool keepObjectInWorld, ros::ServiceClient& client)
{
    dual_arm::MoveGripper moveGripperCall;
    moveGripperCall.request.keepObjectInWorld = keepObjectInWorld;
    moveGripperCall.request.width1.clear();
    if ((robot == M2A::DualArm::Robot::first) || (robot == M2A::DualArm::Robot::dual)) {
        moveGripperCall.request.width1.emplace_back(width);
    }
    moveGripperCall.request.width2.clear();
    if ((robot == M2A::DualArm::Robot::second) || (robot == M2A::DualArm::Robot::dual)) {
        moveGripperCall.request.width2.emplace_back(width);
    }
    if (!client.call(moveGripperCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::MoveGripper!");
    }
}

//! Localizes the taskboard and returns its pose w.r.t. the world frame.
std::optional<Eigen::Isometry3d> Robothon2023::TaskPlanner::localizeTaskBoard(ros::ServiceClient& client)
{
    // Move camera robot to the predefined observation configuration and use the RealSense to localize the taskboard.
    sensor_msgs::JointState observationConfiguration;
    observationConfiguration.position = { -0.0205935, -0.228138, -0.13073, -1.79088, -0.0881729, 1.29869, 0.581941 };
    dual_arm::CoordMoveJS moveCallJS;
    moveCallJS.request.waypoints1.clear();
    moveCallJS.request.waypoints2.clear();
    moveCallJS.request.waypoints2.emplace_back(observationConfiguration);
    if (!client.call(moveCallJS)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::CoordMoveJS!");
        return std::nullopt;
    }
    Robothon2023::TaskPlanner::TaskBoardDetector taskBoardDetector;
    Eigen::Isometry3d boardPlannerRefInBase = taskBoardDetector.getVisionIso3dPose();
    // Hard coded post-processing:
    // Use a vertical z-orientation of the task board.
    Eigen::Quaterniond straight(boardPlannerRefInBase.linear());
    straight.x() = 0.0;
    straight.y() = 0.0;
    straight.normalize();
    boardPlannerRefInBase.linear() = straight.toRotationMatrix();
    // The z-height of the task board is known by the setup. It is placed on a wooden plate with a thickness of 0.0161m.
    boardPlannerRefInBase.translation()[2] = 0.091;
    return { boardPlannerRefInBase };
}

//! Get a transformation matrix that implements an offset in z direction.
Eigen::Isometry3d Robothon2023::TaskPlanner::zOffset(double offset)
{
    Eigen::Isometry3d offsetTransform = Eigen::Isometry3d::Identity();
    offsetTransform.translation().z() = offset;
    return offsetTransform;
}

//! Get the pose for a task w.r.t. the planner reference frame in the corner of the board.
Eigen::Isometry3d Robothon2023::TaskPlanner::taskPose(const std::string& task)
{
    std::vector<double> position;
    std::vector<double> orientation;
    if (!ros::param::get("/task_planner/" + task + "/position", position)
        || !ros::param::get("/task_planner/" + task + "/orientation", orientation)) {
        ROS_ERROR_STREAM(
            "utility_functions.cpp: Could not read pose for task " << task << " from the ROS parameter server.");
    }
    Eigen::Isometry3d taskPose = Eigen::Isometry3d::Identity();
    taskPose.translation()
        = (Eigen::Matrix<double, 3, 1>() << position.at(0), position.at(1), position.at(2)).finished();
    taskPose.linear() = Eigen::Quaterniond(orientation.at(0), orientation.at(1), orientation.at(2), orientation.at(3))
                            .toRotationMatrix();
    return taskPose;
}

//! Calls the server that applies a Cartesian force for a given task.
void Robothon2023::TaskPlanner::force(const M2A::DualArm::Robot& robot, std::string task, ros::ServiceClient& client)
{
    dual_arm::Force forceCall;
    forceCall.request.robotID = static_cast<int>(robot);
    forceCall.request.task.data = task;
    if (!client.call(forceCall)) {
        ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::Force!");
    }
    //    std::thread([&]() {
    //        if (!client.call(forceCall)) {
    //            ROS_ERROR("utility_functions.cpp: Failed to call service dual_arm::Force!");
    //        }
    //    });
}

void Robothon2023::TaskPlanner::forceAction(
    const M2A::DualArm::Robot& robot, std::string task, actionlib::SimpleActionClient<dual_arm::ForceAction>& client)
{
    dual_arm::ForceActionGoal goal;
    goal.goal.task.data = task;
    goal.goal.robotID = static_cast<int>(robot);
    client.sendGoal(goal.goal);
}

//! Compute planar circular path waypoints to open the lid.
std::vector<Eigen::Isometry3d> Robothon2023::TaskPlanner::circleWaypoints(
    const Eigen::Isometry3d& start, double radius, double angle)
{
    constexpr size_t numWaypoints{ 8 };
    // The circle is defined in the x-z-plane. All other values correspond to the first waypoint.
    std::vector<Eigen::Isometry3d> circleWaypoints(numWaypoints, start);
    for (size_t ii = 0; ii < numWaypoints; ++ii) {
        double currentAngle = (M_PI / 180.0) * ii * (angle / numWaypoints); // In radiant.
        circleWaypoints.at(ii).translation().x() -= (radius - cos(currentAngle) * radius);
        circleWaypoints.at(ii).translation().z() += sin(currentAngle) * radius;
    }
    return circleWaypoints;
}
