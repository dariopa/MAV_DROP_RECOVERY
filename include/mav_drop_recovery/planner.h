#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include "mav_drop_recovery/SetTargetPosition.h"

class TrajectoryPlanner {
 public:
  TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  void loadParameters();

  void uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);

  void getFirstPose();

  bool checkPosition(Eigen::Affine3d end_position);

  bool trajectoryPlannerTwoVertices(Eigen::Affine3d end_position, double velocity, double accel);

  bool trajectoryPlannerThreeVertices(Eigen::Affine3d middle_position, Eigen::Affine3d end_position, double velocity, double accel);

  // Service caller for trajectory
  bool trajectoryCallback(mav_drop_recovery::SetTargetPosition::Request& request, 
                          mav_drop_recovery::SetTargetPosition::Response& response);

  // Different trajectories
  bool takeoff();

  bool traverse();

  bool release();

  bool recoveryNet(bool execute);
  
  bool recoveryMagnet(bool execute);

  bool homecoming();

  // Visualize the last planned trajectory
  bool visualizeTrajectory();

  // Output last planned trajectory
  bool executeTrajectory();

  private:
  // Publisher
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;

  // Subscriber
  ros::Subscriber sub_pose_;

  // Services
  ros::ServiceServer trajectory_service_;

  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  Eigen::Affine3d current_position_;
  mav_trajectory_generation::Trajectory trajectory_;

  // Parameters
  Eigen::Affine3d startpoint_; // Startpoint of takeoff
  Eigen::Affine3d checkpoint_; // gives the point in checkPosition() to which the UAV has to be approached
  double safety_altitude_; // meters above take-off height.
  double approach_distance_; // distance from which gps will be approached
  double tolerance_distance_; // used in checkPosition();
  double net_recovery_shift_; // width of recovery net, required for recovery mode
  
  // Configuration Parameters
  double height_uav_gripper_; // Height from uav-antenna to end-effector/ Gripper
  double height_uav_net_; // Height from uav-antenna to lower part of net
  double height_uav_magnet_; // Height from uav-antenna to magnet
  double height_box_antennaplate_; // Height of the antenna plate on the GPS box, in meters
  double height_box_hook_; // Height of a hook on the GPS box, in meters

  // Parameters from Yaml
  double waypoint_1_z_; // takeoff height
  double waypoint_2_x_; // x coord. for traverse
  double waypoint_2_y_; // y coord. for traverse
  double waypoint_3_z_; // release /recovery height
  double v_max_; // m/s
  double a_max_; // m/s^2
  double height_drop_; // variable drop height for release
  double height_overlapping_net_; // [POSITIVE SIGN!] define how much of the net has to overlap with the hook during the recovery
  double height_overlapping_magnet_; // [POSITIVE SIGN!] define how much of the magnet has to overlap with the antennaplate on the box
};