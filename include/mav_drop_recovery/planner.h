#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include "mav_drop_recovery/SetTargetPosition.h"

class TrajectoryPlanner {
 public:
  TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  bool loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void rokubiForceCallback(const geometry_msgs::WrenchStamped& msg);

  void uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose);

  // Service caller for trajectory
  bool trajectoryCallback(mav_drop_recovery::SetTargetPosition::Request& request, 
                          mav_drop_recovery::SetTargetPosition::Response& response);

  // Service client for dynamixel
  bool dynamixelClient(int steps);
  
  void getFirstPose();

  void loadParameters();

  bool checkPositionPayload(Eigen::Affine3d end_position, bool check_recovery_payload=false, bool check_release_payload=false);

  bool trajectoryPlannerTwoVertices(Eigen::Affine3d end_position, double velocity, double accel);

  bool trajectoryPlannerThreeVertices(Eigen::Affine3d middle_position, Eigen::Affine3d end_position, double velocity, double accel);

  // Different trajectories
  bool takeoff();

  bool traverse();

  bool release(bool execute);

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
  ros::Subscriber sub_pose_; // subscribes position of uav
  ros::Subscriber sub_force_; // subscribes force on rokubi sensor

  // Services
  ros::ServiceServer trajectory_service_;
  ros::ServiceServer load_parameters_service_;

  // Clients
  ros::ServiceClient dynamixel_client_;

  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;
  Eigen::Affine3d current_position_; // outputs current position
  double payload_; // [Newton] Measures weight of payload
  mav_trajectory_generation::Trajectory trajectory_;

  // Parameters
  Eigen::Affine3d startpoint_; // Startpoint of takeoff
  Eigen::Affine3d checkpoint_; // gives the point in checkPosition() to which the UAV has to be approached
  double safety_altitude_; // [Meter] meters above take-off height.
  double approach_distance_; // [Meter] distance from which gps will be approached
  double tolerance_distance_; // [Meter] used in checkPosition();
  double net_recovery_shift_; // [Meter] width of recovery net, required for recovery mode
  double height_hovering_; // [Meter] Hovering height above ground after release or recovery
  
  // Coordinate Configuration Parameters
  double height_uav_gripper_; // [Meter] Height from uav-antenna to end-effector/ Gripper
  double height_uav_net_; // [Meter] Height from uav-antenna to lower part of net
  double height_uav_magnet_; // [Meter] Height from uav-antenna to magnet
  double height_box_antennaplate_; // [Meter] Height of the antenna plate on the GPS box, in meters
  double height_box_hook_; // [Meter] Height of a hook on the GPS box, in meters
  double shift_uavantenna_box_x_; // [Meter] Shift in X-direction from antenna to centroid of gps box
  double shift_uavantenna_box_y_; // [Meter] Shift in Y-direction from antenna to the centroid of gps box.
  double payload_threshold_; // [Newton] Threshold which defines if gps box has been released / recovered. 
  double payload_offset_; // [Newton] Offset which is given by the Rokubi sensor. 

  // Parameters from Yaml
  double waypoint_1_z_; // [Meter] takeoff height
  double waypoint_2_x_; // [Meter] x coord. for traverse
  double waypoint_2_y_; // [Meter] y coord. for traverse
  double waypoint_3_z_; // [Meter] release /recovery height
  double v_max_; // [m/s] maximal velociti
  double a_max_; // [m/s^2] maximal acceleration
  double height_drop_; // [Meter] variable drop height for release
  int steps_dynamixel_; // [-] incremental steps of dynamixel to release GPS box
  double height_overlapping_net_; // [Meter] [POSITIVE SIGN!] define how much of the net has to overlap with the hook during the recovery
  double height_overlapping_magnet_; // [Meter] [POSITIVE SIGN!] define how much of the magnet has to overlap with the antennaplate on the box
};