![Robothon 2022 Repository of Team "Wall-E 3.0"](Title_robo-3.png?raw=true "Title")


# Equipment Used List
![Equipment Images](Equipment_Images-3.png?raw=true "Title")
- Franka Emika Panda Robot
- Intel RealSense D435i
- Standard PC with Ubuntu 18.04 (no GPU!)
- Spotlight for repeatable light conditions
- 3D-printed robot attachment for extracting the batteries incl. magnet
- 3D-printed taskboard holder, which holds the taskboard always in the same place after it was pushed in from a random position. Springs prevent the taskboard from moving
- 3D-printed fingers (PETG) with rubber on surface
- Wooden plate as working space
- Checkerboard (printed)
- Charuco Marker (printed)
- Metal Tip (for Hand-Eye-Calibration)

# Software Dependency List
![Software Dependency Logos](logos-3.png?raw=true "Title")
#### Robot (C++)
- Ubuntu 20.04
- Kernel: 5.9.1-rt20
- gcc/g++ 9.4.0
- cmake 3.16.3
- C++20
- ROS Melodic
- Eigen
- franka_ros
- ros-planning / geometric_shapes
- moveit
- rviz
- libfranka

#### Vision (Python)
- Intrinsic camera calibration: ROS camera_calibration
- Tip calibration: custom implementation
- Hand-eye Calibration: ROS easy_handeye
- Hand-eye Calibration Optimization: custom implementation
- 6D Object Pose Estimation of Taskboard: custom implemenation based on ROS planar_pose


# Quick Start Guide
## Build
1. Install *ROS Noetic* as described [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
1. Make sure you have the most up to date packages:
```bash
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
```
1. Install all the packages as listed in the `.gitlab-ci.yml`, e.g. `sudo apt-get install ros-noetic-libfranka` etc.
1. Clone the project with all its submodules: `git clone --recursive https://gitlab.lrz.de/AM/robothon2023.git`
1. Configure the *catkin* workspace and build the project:
```bash
cd robothon2023/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Run the Taskboard
![Screenshot of Project Running](Project_Running-2.png?raw=true "Title")
Open four terminals and navigate to the `catkin_ws` folder in each. Run:
  1. `roslaunch task_planner run_application.launch`
  1. `roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud ordered_pc:=true`
  1. `./src/planar_pose/src/object_detection.py`
  1. `./src/planar_pose/src/planar_pose_estimation.py`

## Calibration
We use the approach described [here](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html) to calibrate the transformation between the `camera_color_optical_frame` of the *Intel RealSense D415* and the `panda_link8` frame of our *Franka Emika* robot.

## References
* The `planar_pose` package is taken based on our `planar_pose` package from our *Robothon 2021* solution and the code and wiki is available [here](https://gitlab.lrz.de/AM/robothon2021). Further information is provided in: *J. Wittmann, F. Pachler, P. Ruhkamp, H. Jung, F. Sygulla and D. Rixen, "Robotic Framework for Autonomous Assembly: a Report from the Robothon 2021 Grand Challenge," 2022 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC), Santa Maria da Feira, Portugal, 2022, pp. 59-65, doi: 10.1109/ICARSC55462.2022.9784775.*
The only modification is the updated target image in `planar_pose/src/objects/*.png` and in `planar_pose/src/config.py`.
Control of First-Order Differential Kinematics“. Manuscript submitted for publication
* The `motion_planner` package is confidential and not included in this repository. Contact jonas.wittmann@tum.de for details. The software architecture is described in: *J.Wittmann, M. Laile, J. Rainer, J. Fottner, and D. Rixen. "Mobile Preassembly Systems with Cooperative Dual-Arm Manipulation - A Concept for Industrial Applications in the Near Future", Manuscript submitted for publication in: ISR Europe 2023; 56th International Symposium on Robotics*
