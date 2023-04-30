/*
 * This file is part of the robothon2023 project.
 * https://gitlab.lrz.de/AM/robothon2023
 */

#include "task_planner/utils/utility_functions.hpp"
#include <Eigen/Geometry>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <array>
#include <chrono>
#include <dual_arm/AddEnvironment.h>
#include <dual_arm/CoordMoveJS.h>
#include <dual_arm/CoordMoveTS.h>
#include <dual_arm/Force.h>
#include <dual_arm/ForceAction.h>
#include <dual_arm/GetPoses.h>
#include <dual_arm/Grasp.h>
#include <dual_arm/MoveGripper.h>
#include <optional>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <thread>
#include <vector>

//! Script executing the tasks for the Robothon 2023 taskboard.
int main(int argc, char* argv[])
{
    // Initialize the ROS node and clients.
    ros::init(argc, argv, "robothon2023_node");
    ros::NodeHandle nh;

    // Initialize the ROS clients.
    ros::ServiceClient coordMoveJSClient = nh.serviceClient<dual_arm::CoordMoveJS>("/dual_arm/coordMoveJS");
    coordMoveJSClient.waitForExistence();
    ros::ServiceClient getPosesClient = nh.serviceClient<dual_arm::GetPoses>("/dual_arm/getEndEffectorPoses");
    getPosesClient.waitForExistence();
    ros::ServiceClient coordMoveTSClient = nh.serviceClient<dual_arm::CoordMoveTS>("/dual_arm/coordMoveTS");
    coordMoveTSClient.waitForExistence();
    ros::ServiceClient addEnvironmentClient = nh.serviceClient<dual_arm::AddEnvironment>("/dual_arm/addEnvironment");
    addEnvironmentClient.waitForExistence();
    ros::ServiceClient graspClient = nh.serviceClient<dual_arm::Grasp>("/dual_arm/grasp");
    graspClient.waitForExistence();
    ros::ServiceClient moveGripperClient = nh.serviceClient<dual_arm::MoveGripper>("/dual_arm/moveGripper");
    moveGripperClient.waitForExistence();
    ros::ServiceClient forceClient = nh.serviceClient<dual_arm::Force>("/dual_arm/force");
    forceClient.waitForExistence();
    actionlib::SimpleActionClient<dual_arm::ForceAction> forceActionClient("/dual_arm/forceAction", true);
    forceActionClient.waitForServer();

    // Wait for all the ROS parameters to be loaded on the ROS Parameter server.
    std::cin.get();
    ros::Rate loopRate(0.5);

    // Hard coded values.
    const Eigen::Isometry3d boardOriginInBoardRef = M2A::DualArm::posRot2mat(
        Eigen::Vector3d(0.06625, 0.1165, -0.043), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).normalized());
    const bool useVision = false; // Use the camera to detect the taskboard.
    const double taskOffset = -0.02; // Offset w.r.t. the TCP for individual tasks.
    const Eigen::Isometry3d customFingerOffset = M2A::DualArm::posRot2mat(
        Eigen::Vector3d(0.0, 0.0, 0.057), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).normalized());

    // Add the static environment model.
    Robothon2023::TaskPlanner::addEnvironmentObject("plate1", addEnvironmentClient);
    Robothon2023::TaskPlanner::addEnvironmentObject("plate2", addEnvironmentClient);
    Robothon2023::TaskPlanner::addEnvironmentObject("storage", addEnvironmentClient);
    Robothon2023::TaskPlanner::addEnvironmentObject("cameraComputer", addEnvironmentClient);
    Robothon2023::TaskPlanner::addEnvironmentObject("controllerBox", addEnvironmentClient);
    Robothon2023::TaskPlanner::addEnvironmentObject("connectionPlate", addEnvironmentClient);

    // The tasks on the board have been defined for the main robot.
    auto toRobot = [](const M2A::DualArm::Robot& robot) {
        return robot == M2A::DualArm::Robot::second
            ? Eigen::Isometry3d::Identity()
            : M2A::DualArm::posRot2mat(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0));
    };

    // Home the robots and open grippers.
    //    Robothon2023::TaskPlanner::home(coordMoveJSClient);
    Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.05, false, moveGripperClient);
    Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::second, 0.05, false, moveGripperClient);

    // Add the taskboard to the environment. It is placed on a wooden plate with a thickness of 0.0161 m.
    const Eigen::Isometry3d boardPlannerRefInBase = useVision
        ? Robothon2023::TaskPlanner::localizeTaskBoard(coordMoveJSClient).value()
        : M2A::DualArm::posRot2mat(
            Eigen::Vector3d(0.456105, -0.138436, 0.091), Eigen::Quaterniond(0.997804, 0, 0, 0.0662368));
    ROS_INFO_STREAM("robothon2023_node.cpp:\nThe taskboard translation w.r.t. base is:\n"
        << boardPlannerRefInBase.translation() << "\nThe taskboard rotation w.r.t. base is:\n"
        << Eigen::Quaterniond(boardPlannerRefInBase.linear()).coeffs());
    const Eigen::Isometry3d boardPlannerRefInWorld
        = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second)
        * boardPlannerRefInBase;
    Robothon2023::TaskPlanner::addEnvironmentObject(
        "task_board", addEnvironmentClient, { boardPlannerRefInWorld * boardOriginInBoardRef });

    // Callable to perform the blue button task.
    auto blueButton = [&](const M2A::DualArm::Robot& robot) {
        Robothon2023::TaskPlanner::moveGripper(robot, 0.0, false, moveGripperClient);
        const Eigen::Isometry3d preBlueButtonInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("blue_button") * toRobot(robot)
            * Robothon2023::TaskPlanner::zOffset(taskOffset);
        std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS> goalPoses = { { std::nullopt, std::nullopt } };
        goalPoses.at(robot) = { preBlueButtonInWorld };
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::force(robot, "button", forceClient);
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::home(coordMoveJSClient);
    };

    // Callable to perform the blue button task.
    auto redButton = [&](const M2A::DualArm::Robot& robot) {
        Robothon2023::TaskPlanner::moveGripper(robot, 0.0, false, moveGripperClient);
        const Eigen::Isometry3d preBlueButtonInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("red_button") * toRobot(robot)
            * Robothon2023::TaskPlanner::zOffset(taskOffset);
        std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS> goalPoses = { { std::nullopt, std::nullopt } };
        goalPoses.at(robot) = { preBlueButtonInWorld };
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::force(robot, "button", forceClient);
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::home(coordMoveJSClient);
    };

    // Callable for plug removal.
    auto plugRemoval = [&](const M2A::DualArm::Robot& robot) {
        Robothon2023::TaskPlanner::moveGripper(robot, 0.0, false, moveGripperClient);
        const Eigen::Isometry3d plugBlackInWorld
            = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("plug_black") * toRobot(robot);
        const Eigen::Isometry3d prePlugBlackInWorld = plugBlackInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS> goalPoses = { { std::nullopt, std::nullopt } };
        goalPoses.at(robot) = { prePlugBlackInWorld };
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::moveGripper(robot, 0.03, false, moveGripperClient);
        goalPoses = { { std::nullopt, std::nullopt } };
        goalPoses.at(robot) = { plugBlackInWorld };
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::grasp({ { { 0.005 }, std::nullopt } }, graspClient, 60.0);
        Robothon2023::TaskPlanner::force(robot, "plug_removal", forceClient);
    };

    // Callable for plug insertion.
    auto plugInsertion = [&](const M2A::DualArm::Robot& robot) {
        const Eigen::Isometry3d prePlugRedInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("plug_red") * toRobot(robot)
            * Robothon2023::TaskPlanner::zOffset(taskOffset);
        std::array<std::optional<Eigen::Isometry3d>, NUM_OF_ROBOTS> goalPoses = { { std::nullopt, std::nullopt } };
        goalPoses.at(robot) = { prePlugRedInWorld };
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::force(robot, "plug_insertion", forceClient);
        Robothon2023::TaskPlanner::moveGripper(robot, 0.03, false, moveGripperClient);
        Robothon2023::TaskPlanner::move(goalPoses, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::home(coordMoveJSClient);
    };

    // Screen detection with second robot; slider with first robot.
    auto sliderTask = [&]() {
        const Eigen::Isometry3d screenDetection
            = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("screen_detection");
        const Eigen::Isometry3d sliderDownInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("slider_down") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d preSliderDownInWorld
            = sliderDownInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        const Eigen::Isometry3d upInDown
            = M2A::DualArm::posRot2mat(Eigen::Vector3d(0.0, 0.013, 0.0), Eigen::Quaterniond(1, 0, 0, 0));


        const Eigen::Isometry3d randInDown
            = M2A::DualArm::posRot2mat(Eigen::Vector3d(0.0, 0.024, 0.0), Eigen::Quaterniond(1, 0, 0, 0));



        const Eigen::Isometry3d sliderUpInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("slider_down") * upInDown * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d preSliderUpInWorld = sliderUpInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        const Eigen::Isometry3d sliderRandInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("slider_down") * randInDown * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d preSliderRandInWorld
            = sliderRandInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        Robothon2023::TaskPlanner::move(
            { { { preSliderDownInWorld }, { screenDetection } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.03, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { { sliderDownInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.011, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { { sliderUpInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { sliderDownInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { sliderUpInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { sliderRandInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.03, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { { preSliderRandInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::home(coordMoveJSClient);
    };

    // Callable for lid opening.
    auto lidOpen = [&]() {
        const Eigen::Isometry3d lidClosedInWorld
            = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second)
            * boardPlannerRefInBase * Robothon2023::TaskPlanner::taskPose("lid_closed");
        const Eigen::Isometry3d lidOpen1InWorld
            = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second)
            * boardPlannerRefInBase * Robothon2023::TaskPlanner::taskPose("lid_open_1");
        const Eigen::Isometry3d lidOpen2InWorld
            = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second)
            * boardPlannerRefInBase * Robothon2023::TaskPlanner::taskPose("lid_open_2");
        const Eigen::Isometry3d lidOpen3InWorld
            = M2A::DualArm::PlannerConfig::get().m_baseInWorldTransform.at(M2A::DualArm::Robot::second)
            * boardPlannerRefInBase * Robothon2023::TaskPlanner::taskPose("lid_open_3");
        Robothon2023::TaskPlanner::move({ { std::nullopt, { lidClosedInWorld } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { std::nullopt, { lidOpen1InWorld } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { std::nullopt, { lidOpen2InWorld } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { std::nullopt, { lidOpen3InWorld } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { (Eigen::Matrix<double, 7, 1>() << 0.0, -0.72, 0.0, -2.05, 0.0, 1.3, 0.7853).finished() } } },
            coordMoveJSClient);
    };

    // Callable for pen task.
    auto pen = [&]() {
        const Eigen::Isometry3d penInWorld
            = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("pen") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d afterPenInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("after_pen") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d afterPen2InWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("after_pen2") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d penInsertion = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("pen_insertion") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d afterPenInsertion = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("after_pen_insertion") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d penStretch = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("pen_stretch")
            * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d prePenInWorld = penInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        Robothon2023::TaskPlanner::move({ { { prePenInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.03, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { { penInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::grasp({ { { 0.008 }, std::nullopt } }, graspClient, 60.0);
        Robothon2023::TaskPlanner::force(M2A::DualArm::Robot::first, "pen_removal", forceClient);
        Robothon2023::TaskPlanner::move({ { { afterPenInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { afterPen2InWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { penInsertion }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::force(M2A::DualArm::Robot::first, "pen_insertion", forceClient);
        Robothon2023::TaskPlanner::move({ { { penInsertion }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { afterPenInsertion }, std::nullopt } }, coordMoveTSClient, false);
        std::vector<Eigen::Matrix<double, 7, 1>> path(0);
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << -0.788868, -0.659608, -0.414391, -1.98407, -0.245731, 1.37103, 1.10233)
                .finished());
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << -0.192147, -0.960809, 0.0222557, -1.57866, 0.260156, 2.13148, 0.863516)
                .finished());
        Robothon2023::TaskPlanner::move(M2A::DualArm::Robot::first, path, coordMoveJSClient);
        Robothon2023::TaskPlanner::move({ { { penStretch }, std::nullopt } }, coordMoveTSClient, false);
    };

    auto cableAndPen = [&]() {
        Robothon2023::TaskPlanner::forceAction(M2A::DualArm::Robot::first, "pen_stretch", forceActionClient);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::second, 0.08, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { std::nullopt,
                                            { (Eigen::Matrix<double, 7, 1>() << -0.365153, -0.603788, 0.517632,
                                                -2.75462, 1.5991, 3.01972, -0.565341)
                                                    .finished() } } },
            coordMoveJSClient);
        const Eigen::Isometry3d cableInWorld1 = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_1");
        const Eigen::Isometry3d cableInWorld2 = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_2");
        Robothon2023::TaskPlanner::move({ { std::nullopt, { cableInWorld1 } } }, coordMoveTSClient, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        Robothon2023::TaskPlanner::move({ { std::nullopt, { cableInWorld2 } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::grasp({ { std::nullopt, { 0.008 } } }, graspClient, 60.0);
        forceActionClient.cancelGoal();
        Robothon2023::TaskPlanner::move({ { { (Eigen::Matrix<double, 7, 1>() << 0.417408, 0.0267058, 0.717913, -2.19408,
                                                0.0616727, 2.34167, -0.191075)
                                                    .finished() },
                                            std::nullopt } },
            coordMoveJSClient);
        Robothon2023::TaskPlanner::move({ { std::nullopt,
                                            { (Eigen::Matrix<double, 7, 1>() << -0.260656, 0.168353, 0.0265677,
                                                -2.12689, 0.036833, 2.34029, 1.93712)
                                                    .finished() } } },
            coordMoveJSClient);
        Robothon2023::TaskPlanner::deleteTaskBoard(addEnvironmentClient);
        Robothon2023::TaskPlanner::move({ { { (Eigen::Matrix<double, 7, 1>() << 0.546737, 0.066282, 0.794375, -0.82743,
                                                0.153029, 2.15847, 0.0605425)
                                                    .finished() },
                                            std::nullopt } },
            coordMoveJSClient);
        Robothon2023::TaskPlanner::move({ { { (Eigen::Matrix<double, 7, 1>() << 0.417408, 0.0267058, 0.717913, -2.19408,
                                                0.0616727, 2.34167, -0.191075)
                                                    .finished() },
                                            std::nullopt } },
            coordMoveJSClient);
        int counter{ 0 };
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_1")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_2")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ++counter;
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_4")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_5")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_6")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_7")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_8")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_9")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_10")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move(
            { { { (Eigen::Matrix<double, 7, 1>() << 0.552541, 0.137824, 0.719923, -1.7448, 0.100196, 1.91213, -0.140763)
                        .finished() },
                std::nullopt } },
            coordMoveJSClient);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
            { { std::nullopt,
                { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_11")
                    * customFingerOffset.inverse() } } },
            coordMoveTSClient, false);
        std::vector<Eigen::Matrix<double, 7, 1>> path(0);
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << 0.443041, -0.36061, 0.41941, -2.03374, 0.650144, 2.43059, 0.174807)
                .finished());
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << 0.375237, -1.1705, 0.196023, -2.62945, 0.650562, 2.40627, 0.382129)
                .finished());
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << -0.332386, -1.61763, -0.114352, -2.84549, 0.328126, 1.85201, 0.488106)
                .finished());
        path.push_back(
            (Eigen::Matrix<double, 7, 1>() << -0.388446, -0.564695, -0.19176, -3.03506, 0.0446047, 2.57348, 0.212695)
                .finished());
        Robothon2023::TaskPlanner::move(M2A::DualArm::Robot::first, path, coordMoveJSClient);
        ROS_INFO_STREAM("Point number " << ++counter << "comes");
        Robothon2023::TaskPlanner::move(
                { { std::nullopt,
                    { boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("cable_first_end_12")
                      * customFingerOffset.inverse() } } },
                coordMoveTSClient, false);
        const Eigen::Isometry3d penBackInsertionInWorld = boardPlannerRefInWorld
            * Robothon2023::TaskPlanner::taskPose("pen_back_insertion") * toRobot(M2A::DualArm::Robot::first);
        const Eigen::Isometry3d prePenBackInsertionInWorld
            = penBackInsertionInWorld * Robothon2023::TaskPlanner::zOffset(taskOffset);
        Robothon2023::TaskPlanner::move({ { { prePenBackInsertionInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { penBackInsertionInWorld }, std::nullopt } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::force(M2A::DualArm::Robot::first, "pen_back_insertion", forceClient);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::first, 0.05, false, moveGripperClient);
        Robothon2023::TaskPlanner::moveGripper(M2A::DualArm::Robot::second, 0.08, false, moveGripperClient);
        Robothon2023::TaskPlanner::move({ { { prePenBackInsertionInWorld }, std::nullopt } }, coordMoveTSClient, false);
        dual_arm::GetPoses getPosesCall;
        if (!getPosesClient.call(getPosesCall)) {
            ROS_ERROR("robothon2023_node.cpp: Server not reached");
        }
        const Eigen::Isometry3d firstEE{ M2A::DualArm::ros2eigen(
            getPosesCall.response.poses.at(M2A::DualArm::Robot::first)) };
        const Eigen::Isometry3d secondEE{ M2A::DualArm::ros2eigen(
            getPosesCall.response.poses.at(M2A::DualArm::Robot::second)) };
        const Eigen::Isometry3d backMotion = M2A::DualArm::posRot2mat(
            Eigen::Vector3d(0.0, 0.0, -0.1), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).normalized());
        Robothon2023::TaskPlanner::move(
            { { { firstEE * backMotion }, { secondEE * backMotion } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::move({ { { (Eigen::Matrix<double, 7, 1>() << -1.3389, -0.508682, 0.0267882, -2.75083,
                                                -0.0631318, 2.25868, -0.507078)
                                                    .finished() },
                                            std::nullopt } },
            coordMoveJSClient);
        if (!getPosesClient.call(getPosesCall)) {
            ROS_ERROR("robothon2023_node.cpp: Server not reached");
        }
        const Eigen::Isometry3d secondEE2{ M2A::DualArm::ros2eigen(
            getPosesCall.response.poses.at(M2A::DualArm::Robot::second)) };
        const Eigen::Isometry3d backMotion2 = M2A::DualArm::posRot2mat(
            Eigen::Vector3d(-0.1, -0.10, 0.0), Eigen::Quaterniond(0.966, 0.259, 0.0, 0.0).normalized());
        Robothon2023::TaskPlanner::move({ { std::nullopt, { secondEE2 * backMotion2 } } }, coordMoveTSClient, false);
        Robothon2023::TaskPlanner::home(coordMoveJSClient);
    };

    blueButton(M2A::DualArm::Robot::first);
    plugRemoval(M2A::DualArm::Robot::first);
    plugInsertion(M2A::DualArm::Robot::first);
    sliderTask();
    lidOpen();
    pen();



//    const Eigen::Isometry3d penStretch = boardPlannerRefInWorld * Robothon2023::TaskPlanner::taskPose("pen_stretch")
//                                         * toRobot(M2A::DualArm::Robot::first);
//    Robothon2023::TaskPlanner::move({ { { penStretch }, std::nullopt } }, coordMoveTSClient, false);
//    std::cin.get();
//    Robothon2023::TaskPlanner::grasp({ { { 0.008 }, std::nullopt } }, graspClient, 60.0);


    cableAndPen();
    redButton(M2A::DualArm::Robot::first);

    ros::spinOnce();
    loopRate.sleep();
    return EXIT_SUCCESS;
}
