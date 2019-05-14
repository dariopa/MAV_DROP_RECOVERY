#include <mav_drop_recovery/planner.h>

TrajectoryPlanner::TrajectoryPlanner(ros::NodeHandle& nh, ros::NodeHandle& nh_private) :
    nh_(nh),
    nh_private_(nh_private),
    safety_altitude_(2.5),
    approach_distance_(1.0),
    tolerance_distance_(0.05),
    net_recovery_shift_(0.3),
    height_hovering_(1.5),
    height_uav_gripper_(0.2),
    height_uav_net_(0.4),
    height_uav_magnet_(0.6),
    height_box_antennaplate_(0.08),
    height_box_hook_(0.18),
    shift_uavantenna_box_x_(0.1),
    shift_uavantenna_box_y_(0.2),
    payload_threshold_(8.0),
    payload_offset_(26),
    current_position_(Eigen::Affine3d::Identity()) {

  // create publisher for RVIZ markers
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
  pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

  // subscriber for pose
  sub_pose_ = nh.subscribe("uav_pose", 1, &TrajectoryPlanner::uavPoseCallback, this);

  // subscriber for rokubi force
  sub_force_ = nh.subscribe("/rokubi_210_node/force_torque_sensor_measurements", 1, &TrajectoryPlanner::rokubiForceCallback, this);

  // trajectory server
  trajectory_service_ = nh.advertiseService("trajectory", &TrajectoryPlanner::trajectoryCallback, this);
  load_parameters_service_ = nh.advertiseService("load_parameters", &TrajectoryPlanner::loadParametersCallback, this);

  // dynamixel client
  dynamixel_client_ = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
}

bool TrajectoryPlanner::loadParametersCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  loadParameters();
  return true;
}

void TrajectoryPlanner::rokubiForceCallback(const geometry_msgs::WrenchStamped& msg) {
  payload_ = msg.wrench.force.z - payload_offset_;
  // ROS_WARN("%f", payload_);
  // ros::Duration(1.0).sleep();
}

void TrajectoryPlanner::uavPoseCallback(const geometry_msgs::Pose::ConstPtr& pose) {
  tf::poseMsgToEigen(*pose, current_position_);
}

bool TrajectoryPlanner::trajectoryCallback(mav_drop_recovery::SetTargetPosition::Request& request, 
                                           mav_drop_recovery::SetTargetPosition::Response& response) {
  
  bool function_execute = false; // if trajectory-function returns false, then it shall not be executed
  // TAKEOFF                                          
  if (request.command == "takeoff") {
    function_execute = takeoff();
  }
  // TRAVERSE
  else if (request.command == "traverse") {
    function_execute = traverse();
  }
  // RELEASE
  else if (request.command == "release") {
    release(request.execute);
  }
  // RECOVERY WITH NET
  else if (request.command == "recovery_net") {
    recoveryNet(request.execute); // we don't want to send execution, as we will execute in the function itself
  }
  // RECOVERY WITH MAGNET
  else if (request.command == "recovery_magnet") {
    recoveryMagnet(request.execute);
  }
  // HOMECOMING
  else if (request.command == "homecoming") {
    function_execute = homecoming();
  }
  else {
    ROS_WARN("INCORRECT_INPUT - CHECK AND RETRY");
    response.success == false;
    return false; 
  }

  // Check if trajectory execution is demanded.
  if (request.execute == true && function_execute == true) {
    executeTrajectory();
    checkPositionPayload(checkpoint_);
  }

  response.success == true;
  return true;
}

bool TrajectoryPlanner::dynamixelClient(int steps) {
  dynamixel_workbench_msgs::DynamixelCommand srv;
  srv.request.command = "";
  srv.request.id = 1;
  srv.request.addr_name = "Goal_Position";
  srv.request.value = steps;
  if (dynamixel_client_.call(srv)) {
    ROS_WARN("DYNAMIXEL CALL SUCCESSFULL.");
    return true;
  }
  else {
    ROS_WARN("DYNAMIXEL CALL UNSUCCESSFULL.");
    return false;
  }
}

void TrajectoryPlanner::getFirstPose() {
  startpoint_.translation() = current_position_.translation();
}

void TrajectoryPlanner::loadParameters() {
  CHECK(nh_private_.getParam("wp1_z", waypoint_1_z_) &&
        nh_private_.getParam("wp2_x", waypoint_2_x_) && 
        nh_private_.getParam("wp2_y", waypoint_2_y_) && 
        nh_private_.getParam("wp3_z", waypoint_3_z_) &&
        nh_private_.getParam("v_max", v_max_) &&
        nh_private_.getParam("a_max", a_max_) &&
        nh_private_.getParam("height_drop", height_drop_) &&
        nh_private_.getParam("steps_dynamixel", steps_dynamixel_) &&
        nh_private_.getParam("height_overlapping_net", height_overlapping_net_) &&
        nh_private_.getParam("height_overlapping_magnet", height_overlapping_magnet_))
        << "Error loading parameters!";
  
  // Configure end position to antenna of uav. 
  waypoint_2_x_ -= shift_uavantenna_box_x_;
  waypoint_2_y_ -= shift_uavantenna_box_y_;
  ROS_WARN("PARAMETERS LOADED!");
}

bool TrajectoryPlanner::checkPositionPayload(Eigen::Affine3d end_position, bool check_recovery_payload, bool check_release_payload) {
  double distance_to_goal; // distance between acutal position and goal-position of drone in checkPositionPayload()
  while(true) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
    
    // Check distance to desired goal position
    distance_to_goal = sqrt(pow(current_position_.translation().x() - end_position.translation().x(), 2) + 
                            pow(current_position_.translation().y() - end_position.translation().y(), 2) + 
                            pow(current_position_.translation().z() - end_position.translation().z(), 2));
    if (distance_to_goal <= tolerance_distance_) {
      ROS_WARN("TRAJECTORY TERMINATED.");
      return true;
    }

    // Check if you picked up something during recovery
    if (check_recovery_payload && payload_ > payload_threshold_) {
      ROS_WARN("YOU GRABBED ON GPS BOX - STOPPING TRAJECTORY!");
      end_position = current_position_;
      end_position.translation().z() += 0.01; // Arbitrary value > 0, to be able to generate a trajectory
      trajectoryPlannerTwoVertices(end_position, v_max_*0.05, a_max_*0.1);
      executeTrajectory();
      return true;
    }

    // Check if you touched the ground during the release
    if (check_release_payload && payload_ < payload_threshold_) {
      ROS_WARN("YOU LANDED ON THE GROUND - STOPPING TRAJECTORY!");
      end_position = current_position_;
      end_position.translation().z() += 0.01; // Arbitrary value > 0, to be able to generate trajectory
      trajectoryPlannerTwoVertices(end_position, v_max_*0.05, a_max_*0.1);
      executeTrajectory();
      return true;
    }
  }
}

bool TrajectoryPlanner::trajectoryPlannerTwoVertices(Eigen::Affine3d end_position, double velocity, double accel) {
  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 2 vertices: start = current position || end = Final point.
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = end_position.translation().x();
  end_point_position.y() = end_position.translation().y();
  end_point_position.z() = end_position.translation().z();
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, velocity, accel);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeTrajectory();
  return true;
}

bool TrajectoryPlanner::trajectoryPlannerThreeVertices(Eigen::Affine3d middle_position, Eigen::Affine3d end_position, double velocity, double accel) {
  mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  // we have 3 vertices: start = current position | middle = intermediate position | end = Final point.
  mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

  // set start point constraints
  // (current position, and everything else zero)
  start.makeStartOrEnd(current_position_.translation(), derivative_to_optimize);
  vertices.push_back(start);

  // set middle point constraints
  Eigen::Vector3d middle_point_position;
  middle_point_position.x() = middle_position.translation().x();
  middle_point_position.y() = middle_position.translation().y();
  middle_point_position.z() = middle_position.translation().z();
  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, middle_point_position);
  vertices.push_back(middle);

  // plan final point if needed (to end position at rest).
  Eigen::Vector3d end_point_position = current_position_.translation();
  end_point_position.x() = end_position.translation().x();
  end_point_position.y() = end_position.translation().y();
  end_point_position.z() = end_position.translation().z();
  end.makeStartOrEnd(end_point_position, derivative_to_optimize);
  vertices.push_back(end);

  // compute segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, velocity, accel);

  // solve trajectory
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();

  // get trajectory
  trajectory_.clear();
  opt.getTrajectory(&trajectory_);
  
  visualizeTrajectory();
  return true;
}

bool TrajectoryPlanner::takeoff() {
  Eigen::Affine3d waypoint_takeoff = current_position_;

  // check if actually a takeoff, i.e. check if drone wants to collide to ground
  if (waypoint_1_z_ < startpoint_.translation().z()) {
    ROS_WARN("NOT A TAKEOFF, YOU CRASH INTO THE GROUND - NOT EXECUTING!");
    return false;
  }
  // check if takeoff altitude is above safety altitude
  if (waypoint_1_z_ < startpoint_.translation().z() + safety_altitude_) {
    ROS_WARN("TAKE OFF TOO LOW. INCREASE TAKE OFF ALTITUDE - NOT EXECUTING!");
    return false;
  }
  // only conduce takeoff when really in takeoff area, which is somewhere around 1 meters distant from the startpoint
  if (abs(waypoint_takeoff.translation().x() - startpoint_.translation().x()) > 1.0 ||
      abs(waypoint_takeoff.translation().y() - startpoint_.translation().y()) > 1.0 ) { 
    ROS_WARN("YOU'RE NOT IN THE TAKEOFF REGION - NOT EXECUTING!");
    return false;
  }

  // if checks are done, then takeoff
  waypoint_takeoff.translation().z() = waypoint_1_z_; 
  checkpoint_ = waypoint_takeoff;
  trajectoryPlannerTwoVertices(waypoint_takeoff, v_max_*0.5, a_max_);
  return true;
}

bool TrajectoryPlanner::traverse() {
  Eigen::Affine3d waypoint_traverse = current_position_; 

  // check if you're high enough for traversation, i.e. if you're above safety altitude
  if (waypoint_traverse.translation().z() < startpoint_.translation().z() + safety_altitude_) {
    ROS_WARN("YOU'RE NOT ON THE TRAVERSATION HEIGHT - NOT EXECUTING!");
    return false;
  }
  
  // if checks are done, then traverse
  waypoint_traverse.translation().x() = waypoint_2_x_; 
  waypoint_traverse.translation().y() = waypoint_2_y_;
  checkpoint_ = waypoint_traverse;
  trajectoryPlannerTwoVertices(waypoint_traverse, v_max_, a_max_);
  return true;
}

bool TrajectoryPlanner::release(bool execute) {
  Eigen::Affine3d waypoint_descend = current_position_;

  // only conduce release when really in release area, which is somewhere around 1 meters distant from the release point
  if (abs(waypoint_descend.translation().x() - waypoint_2_x_) > 1.0 ||
      abs(waypoint_descend.translation().y() - waypoint_2_y_) > 1.0 ) { 
    ROS_WARN("YOU'RE NOT IN THE RELEASE REGION - NOT EXECUTING!");
    return false;
  }
  // if checks are all good, then release
  // Descend
  waypoint_descend.translation().z() = waypoint_3_z_ + height_box_antennaplate_ + height_uav_gripper_ + height_drop_;
  trajectoryPlannerTwoVertices(waypoint_descend, v_max_*0.2, a_max_);
  if (execute) {
    executeTrajectory();
    ROS_WARN("DESCENDING.");
    checkPositionPayload(waypoint_descend, false, true);
  }

  // Engage Dynamixel
  dynamixelClient(steps_dynamixel_);
  ros::Duration(5.0).sleep(); 

  // Ascend
  Eigen::Affine3d waypoint_ascend = waypoint_descend;
  waypoint_ascend.translation().z() = waypoint_3_z_ + height_hovering_;
  trajectoryPlannerTwoVertices(waypoint_ascend, v_max_*0.3, a_max_);
  if (execute) {
    executeTrajectory();
    ROS_WARN("ASCENDING.");
    checkPositionPayload(waypoint_ascend);
  }

  // Check if gps box has been detached
  if (payload_ < payload_threshold_) {
    ROS_WARN("GPS BOX IS DETACHED!");
  }
  else {
    ROS_WARN("PROBLEM WHILE DETACHING GPS BOX - RETRY!");
  }

  // Reposition Dynamixel
  dynamixelClient(10); 

  return true;
}

bool TrajectoryPlanner::recoveryNet(bool execute) {
  Eigen::Affine3d waypoint_recovery = current_position_;
  // only conduce recovery when really in recovery area, which is somewhere around 1 meters distant from the recovery point
  if (abs(waypoint_recovery.translation().x() - waypoint_2_x_) > 1.0 ||
      abs(waypoint_recovery.translation().y() - waypoint_2_y_) > 1.0 ) { 
    ROS_WARN("YOU'RE NOT IN THE RECOVERY REGION - NOT EXECUTING!");
    return false;
  }

  // if checks are all good, then recover
  int direction_change = 1;
  for (int counter = 0; counter <= 2; counter++) {
    // First slightly return on the x-axis
    Eigen::Affine3d waypoint_one = current_position_;
    waypoint_one.translation().x() -= approach_distance_;
    if (counter > 0) {
      waypoint_one.translation().x() -= approach_distance_; // If re-iteration required, then enter this if-condition again
    }
    waypoint_one.translation().y() += (net_recovery_shift_ * counter * direction_change);
    trajectoryPlannerTwoVertices(waypoint_one, v_max_*0.5, a_max_);
    if (execute) {
      executeTrajectory();
      ROS_WARN("STEPPING BACK ON TRAVERSATION PATH.");
      checkPositionPayload(waypoint_one);
    }

    // Then, go down on pickup height
    Eigen::Affine3d waypoint_two = waypoint_one;
    waypoint_two.translation().z() = waypoint_3_z_ + height_box_hook_ + height_uav_net_ - height_overlapping_net_;
    trajectoryPlannerTwoVertices(waypoint_two, v_max_ * 0.2, a_max_);
    if (execute) {
      executeTrajectory();
      ROS_WARN("DESCENDING ON APPROACHING POSITION.");
      checkPositionPayload(waypoint_two);
    }

    // Pick up GPS box
    Eigen::Affine3d waypoint_three = waypoint_two;
    waypoint_three.translation().x() = waypoint_2_x_ + approach_distance_;
    trajectoryPlannerTwoVertices(waypoint_three, v_max_*0.1, a_max_);
    if (execute) {
      executeTrajectory();
      ROS_WARN("PICKING UP GPS BOX.");
      if (checkPositionPayload(waypoint_three, true, false)) {
        break; // Go out of for-loop in checkPositionPayload()!
      }
    }

    // Elevate with GPS box
    Eigen::Affine3d waypoint_four = waypoint_three;
    waypoint_four.translation().z() = waypoint_3_z_ + height_hovering_;
    trajectoryPlannerTwoVertices(waypoint_four, v_max_*0.2, a_max_);
    if (execute) {
      executeTrajectory();
      ROS_WARN("ELEVATING.");
      checkPositionPayload(waypoint_four);
    }

    // Check if you loaded the GPS Box
    if (payload_ > payload_threshold_) {
      ROS_WARN("GPS BOX SUCCESSFULLY PICKED UP!");
      break;
    }
    else {
      ROS_WARN("GPS BOX MISSED -RETRY!");
      direction_change *= -1;
    }
  }

  // Getting ready for homecoming
  Eigen::Affine3d waypoint_five;
  waypoint_five.translation().x() = waypoint_2_x_;
  waypoint_five.translation().y() = waypoint_2_y_;
  waypoint_five.translation().z() = waypoint_1_z_;
  trajectoryPlannerTwoVertices(waypoint_five, v_max_*0.3, a_max_);
  if (execute) {
    executeTrajectory();
    ROS_WARN("GO TO POSITION FOR HOMECOMING.");
    checkPositionPayload(waypoint_five);
  }
  return true;
}

bool TrajectoryPlanner::recoveryMagnet(bool execute) {
  Eigen::Affine3d waypoint_recovery = current_position_;
  // only conduce recovery when really in recovery area, which is somewhere around 1 meters distant from the recovery point
  if (abs(waypoint_recovery.translation().x() - waypoint_2_x_) > 1.0 ||
      abs(waypoint_recovery.translation().y() - waypoint_2_y_) > 1.0 ) { 
    ROS_WARN("YOU'RE NOT IN THE RECOVERY REGION - NOT EXECUTING!");
    return false;
  }
  // if checks are all good, then release
  // Pickup with magnet: first descend
  Eigen::Affine3d waypoint_one = current_position_;
  waypoint_one.translation().z() = waypoint_3_z_ + height_box_antennaplate_ + height_uav_magnet_ - height_overlapping_magnet_;
  trajectoryPlannerTwoVertices(waypoint_one, v_max_*0.3, a_max_);
  if (execute) {
    executeTrajectory();
    ROS_WARN("DESCENDING.");
    checkPositionPayload(waypoint_one);
  }

  // Pickup with magnet: second ascend (if weight has increased)
  Eigen::Affine3d waypoint_two = current_position_;
  waypoint_two.translation().z() = waypoint_3_z_ + height_hovering_;
  trajectoryPlannerTwoVertices(waypoint_two, v_max_*0.3, a_max_);
  if (execute) {
    executeTrajectory();
    ROS_WARN("ASCENDING.");
    checkPositionPayload(waypoint_two);
  }

  // Check if you loaded the GPS Box
  if (payload_ > payload_threshold_) {
    ROS_WARN("GPS BOX SUCCESSFULLY PICKED UP!");
    return true;
  }
  else {
    ROS_WARN("GPS BOX MISSED -RETRY!");
    return false;
  }
}

bool TrajectoryPlanner::homecoming() {
  // check if you're already home or not
  if (abs(current_position_.translation().x() - startpoint_.translation().x()) < 1.0 &&
      abs(current_position_.translation().y() - startpoint_.translation().y()) < 1.0 ) { 
    ROS_WARN("YOU'RE ALREADY HOME - NOT EXECUTING!");
    return false;
  }

  Eigen::Affine3d waypoint_homecoming_middle; 
  waypoint_homecoming_middle.translation().x() = (current_position_.translation().x() + startpoint_.translation().x()) / 2;
  waypoint_homecoming_middle.translation().y() = (current_position_.translation().y() + startpoint_.translation().y()) / 2;
  waypoint_homecoming_middle.translation().z() = waypoint_1_z_;

  Eigen::Affine3d waypoint_homecoming_end;
  waypoint_homecoming_end.translation().x() = startpoint_.translation().x();
  waypoint_homecoming_end.translation().y() = startpoint_.translation().y();
  waypoint_homecoming_end.translation().z() = (startpoint_.translation().z() + 2.0);

  checkpoint_ = waypoint_homecoming_end;
  trajectoryPlannerThreeVertices(waypoint_homecoming_middle, waypoint_homecoming_end, v_max_ * 0.5, a_max_);
  return true;
}

bool TrajectoryPlanner::visualizeTrajectory() {
  visualization_msgs::MarkerArray markers;
  double distance = 0.3; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory_,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);
  return true;
}

bool TrajectoryPlanner::executeTrajectory() {
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory_, &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);
  return true;
}
