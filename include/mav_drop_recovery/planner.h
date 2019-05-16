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

  bool homeComing();

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

  // Variable Parameters
  Eigen::Affine3d current_position_; // outputs current position
  Eigen::Affine3d checkpoint_; // gives the position in checkPosition() to which the UAV has to be approached
  double payload_; // [Newton] Measures weight of payload
  mav_trajectory_generation::Trajectory trajectory_;

  // Constant Parameters
  Eigen::Affine3d startpoint_; // Startpoint of takeoff
  double safety_altitude_; // [Meter] meters above take-off height.
  double approach_distance_; // [Meter] distance from which gps will be approached
  double tolerance_distance_; // [Meter] control distance to check if you arrived at the desired goal position
  double start_trajectory_distance_; // [Meter] won't let you start trajectory if: current_positon_ > start_position + start_trajectory_distance_
  double net_recovery_shift_; // [Meter] width of recovery net, required for recovery mode
  double height_hovering_; // [Meter] Hovering height above ground after release or recovery
  double payload_threshold_; // [Newton] Threshold which defines if gps box has been released / recovered. 
  double payload_offset_; // [Newton] Offset in z-direction which is given by the Rokubi sensor.
  
  // Z-Coordinate Configuration Parameters
  double height_box_antennaplate_; // [Meter] Height from ground to the antenna plate on the GPS box
  double height_box_hook_; // [Meter] Height from ground to a hook on the GPS box
  double height_rokubi_gripper_; // [Meter] Height from rokubi to downside of gripper
  double height_rokubi_net_; // [Meter] Height from rokubi to downside of net
  double height_rokubi_magnet_; // [Meter] Height from rokubi to downside of magnet 

  // Yaml - Configuration Parameters
  double waypoint_1_z_; // [Meter] takeoff altitude
  double waypoint_2_x_; // [Meter] Longitude for traverse
  double waypoint_2_y_; // [Meter] Latitude for traverse
  double waypoint_3_z_; // [Meter] release / recovery height
  double v_max_; // [m/s] maximal velocity
  double a_max_; // [m/s^2] maximal acceleration
  double v_scaling_descending_; // [-] scaling factor for velocity when descending
  double v_scaling_ascending_; // [-] scaling factor for velocity when ascending
  double v_scaling_recovery_traverse_; // [-] scaling factor for velocity when recovering box with net
  double v_scaling_general_traverse_; // [-] scaling factof for velocity for general traversation 
  double height_drop_; // [Meter] variable drop height for release
  int steps_dynamixel_; // [-] incremental steps of dynamixel to release GPS box
  double height_overlapping_net_; // [Meter] [POSITIVE SIGN!] define how much of the net has to overlap with the hook during the recovery
  double height_overlapping_magnet_; // [Meter] [POSITIVE SIGN!] define how much of the magnet has to overlap with the antennaplate on the box
  Eigen::Affine3d transformation_uav_rokubi_; // Transformation from uav GPS antenna to the lower attachment side of Rokubi
};