#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <geometry_msgs/Pose.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ruckig/ruckig.hpp>
#include <trac_ik/trac_ik.hpp>

// #include "cnpy.h"
#include "common.h"
#include "panda_interface.h"
// #include "panda_sim_real_interface/JointDataArray.h"
// #include "panda_sim_real_interface/RobotPlan.h"
#include "ros/ros.h"

std::string URDF_PATH =
    "/home/gilberto/npm/catkin_ws/src/panda_sim_real_interface/assets/"
    "panda.urdf";
ruckig::Ruckig<5> ruckig_5(0.001);
ruckig::Ruckig<6> ruckig_6(0.001);
ruckig::Ruckig<7> ruckig_7(0.001);

void PandaInterface::initRobot() {
  std::cout << "Initializing Controller!" << std::endl;
  robot_ = std::make_shared<franka::Robot>(franka_address_);
  robot_->setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  robot_model_ = std::make_shared<franka::Model>(robot_->loadModel());
}

ruckig::InputParameter<5> PandaInterface::get_ruckig_input_5(
    std::array<double, 5> inital_q, std::array<double, 5> inital_q_vel,
    std::array<double, 5> inital_q_acc, std::array<double, 5> target_q,
    std::array<double, 5> target_q_vel, std::array<double, 5> target_q_acc) {
  ruckig::InputParameter<5> input;
  input.current_position = inital_q;
  input.current_velocity = inital_q_vel;
  input.current_acceleration = inital_q_acc, input.target_position = target_q;
  input.target_velocity = target_q_vel;
  input.target_acceleration = target_q_acc;
  // limits from https://frankaemika.github.io/docs/control_parameters.html
  input.max_velocity = {2.1750 / 2, 2.1750 / 2, 2.6100 / 2, 2.6100 / 2,
                        2.1600 / 2};
  input.max_acceleration = {10 / 2, 12.5 / 2, 15 / 2, 20 / 2, 20 / 2};
  input.max_jerk = {3000 / 2, 3250 / 2, 4500 / 2, 6000 / 2, 6000 / 2};
  return input;
}

ruckig::Trajectory<5> PandaInterface::get_ruckig_trajectory_5(
    std::array<double, 7> initial_q_7, std::array<double, 7> target_q_7) {
  std::array<double, 5> initial_q = {initial_q_7[2], initial_q_7[3],
                                     initial_q_7[4], initial_q_7[5],
                                     initial_q_7[6]};
  std::array<double, 5> target_q = {target_q_7[2], target_q_7[3], target_q_7[4],
                                    target_q_7[5], target_q_7[6]};

  std::array<double, 5> initial_q_vel = {0, 0, 0, 0, 0};
  std::array<double, 5> target_q_vel = {0, 0, 0, 0, 0};
  std::array<double, 5> initial_q_acc = {0, 0, 0, 0, 0};
  std::array<double, 5> target_q_acc = {0, 0, 0, 0, 0};
  ruckig::InputParameter<5> input =
      get_ruckig_input_5(initial_q, initial_q_vel, initial_q_acc, target_q,
                         target_q_vel, target_q_acc);
  ruckig::OutputParameter<5> output;
  ruckig::Trajectory<5> trajectory;
  ruckig::Result result = ruckig_5.calculate(input, trajectory);
  return trajectory;
}

ruckig::InputParameter<7> PandaInterface::get_ruckig_input(
    std::array<double, 7> inital_q, std::array<double, 7> inital_q_vel,
    std::array<double, 7> inital_q_acc, std::array<double, 7> target_q,
    std::array<double, 7> target_q_vel, std::array<double, 7> target_q_acc) {
  ruckig::InputParameter<7> input;
  input.current_position = inital_q;
  input.current_velocity = inital_q_vel;
  input.current_acceleration = inital_q_acc, input.target_position = target_q;
  input.target_velocity = target_q_vel;
  // input.target_acceleration = target_q_acc;
  //  limits from https://frankaemika.github.io/docs/control_parameters.html
  input.max_velocity = {2.1750 / 2, 2.1750 / 2, 2.1750 / 2, 2.1750 / 2,
                        2.6100 / 2, 2.6100 / 2, 2.1600 / 2};
  input.max_acceleration = {15 / 2, 7.5 / 2, 10 / 2, 12.5 / 2,
                            15 / 2, 20 / 2,  20 / 2};
  input.max_jerk = {5500, 2000, 3000, 3250, 4500, 6000, 6000};
  return input;
}
ruckig::InputParameter<6> PandaInterface::get_ruckig_input_cartesian(
    std::array<double, 6> inital_q, std::array<double, 6> inital_q_vel,
    std::array<double, 6> inital_q_acc, std::array<double, 6> target_q,
    std::array<double, 6> target_q_vel, std::array<double, 6> target_q_acc) {
  ruckig::InputParameter<6> input;
  input.current_position = inital_q;
  input.current_velocity = inital_q_vel;
  input.current_acceleration = inital_q_acc, input.target_position = target_q;
  input.target_velocity = target_q_vel;
  // input.target_acceleration = target_q_acc;
  //  limits from https://frankaemika.github.io/docs/control_parameters.html
  input.max_velocity = {1.7 / 4,    1.7 / 4,    1.7 / 4,
                        2.1750 / 4, 2.1750 / 4, 2.1750 / 4};
  input.max_acceleration = {13 / 10, 13 / 10, 13 / 10,
                            10 / 10, 10 / 10, 10 / 10};
  input.max_jerk = {5500 / 10, 5500 / 10, 5500 / 10,
                    3000 / 10, 3000 / 10, 3000 / 10};
  return input;
}
void PandaInterface::perform_edge(
    std::array<double, 7> initial_q, std::array<double, 7> target_q,
    franka::Robot *robot,
    std::tuple<std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<double, NUM_STEPS>, std::array<double, 2>, int>
        *trajectory_data) {
  move_to_joint_angles(robot, initial_q);
  ruckig::Trajectory<5> trajectory =
      get_ruckig_trajectory_5(initial_q, target_q);
  try {
    move_failure(robot, &trajectory, 1, trajectory_data);
  } catch (const franka::Exception &e) {
    std::cout << "Exception: " << e.what() << std::endl;
    std::array<std::array<double, 7>, NUM_STEPS> empty_array;
    std::array<std::array<double, 7>, NUM_STEPS> empty_array2;
    std::array<std::array<double, 7>, NUM_STEPS> empty_array3;
    std::array<double, NUM_STEPS> empty_array4;
    std::array<double, 2> empty_array5;
    int empty_int = 0;
    *trajectory_data = std::make_tuple(empty_array, empty_array2, empty_array3,
                                       empty_array4, empty_array5, empty_int);
  }
}
// put tested functions in here
void PandaInterface::set_joint_and_collision_behaviour(franka::Robot *robot) {
  /*
  Sets the joint and collision behaviour of the robot
  */
  robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  std::array<double, 7> lower_torque_thresholds_nominal{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
  std::array<double, 7> upper_torque_thresholds_nominal{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 7> lower_torque_thresholds_acceleration{
      {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
  std::array<double, 7> upper_torque_thresholds_acceleration{
      {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
  std::array<double, 6> lower_force_thresholds_nominal{
      {30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_nominal{
      {40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  std::array<double, 6> lower_force_thresholds_acceleration{
      {30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
  std::array<double, 6> upper_force_thresholds_acceleration{
      {40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
  robot->setCollisionBehavior(
      lower_torque_thresholds_acceleration,
      upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
      upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
      upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
      upper_force_thresholds_nominal);
}

void PandaInterface::move_to_default_pose(franka::Robot *robot) {
  /*
  Moves the robot to the default pose
  */
  std::array<double, 7> q_default_goal = {0, -M_PI_4, 0,     -3 * M_PI_4,
                                          0, M_PI_2,  M_PI_4};
  MotionGenerator motion_generator(0.5, q_default_goal);  // speed factor, goal
  robot->control(motion_generator);
}

void PandaInterface::move_to_joint_angles(franka::Robot *robot,
                                          std::array<double, 7> q_goal) {
  /*
  Moves the robot to the specified joint angles
  */
  MotionGenerator motion_generator(0.5, q_goal);  // speed factor, goal
  robot->control(motion_generator);
}

int PandaInterface::get_closest_time(float time,
                                     std::vector<double> time_list) {
  /*
  Returns the index of the closest time in the time_list
  */
  int closest_time_index = 0;
  float closest_time = 1000000;
  for (int i = 0; i < time_list.size(); i++) {
    if (std::abs(time_list[i] - time) < closest_time) {
      closest_time = std::abs(time_list[i] - time);
      closest_time_index = i;
    }
  }
  return closest_time_index;
}

successJointAngles PandaInterface::ik(std::array<double, 7> pose) {
  /*
   * Get the joint angles for a given pose using trac_ik
   * @param pose: 7 element array of doubles representing the pose of the end
   * effector in the form [x, y, z, qx, qy, qz, qw]
   * @param urdf_path: path to urdf file
   * @return: a struct containing a bool representing whether the ik was
   * successful and a 7 element array of doubles representing the joint angles
   */
  // load robot model from urdf
  std::string name = "panda";
  successJointAngles s;
  bool success = true;

  // create tree from urdf_string
  KDL::Tree tree;
  kdl_parser::treeFromFile(URDF_PATH, tree);

  // create chain from tree
  KDL::Chain chain;
  tree.getChain(name + "_link0", name + "_hand", chain);
  // std::cout << "Num Joints in chain: " << chain.getNrOfJoints() << std::endl;

  std::array<double, 7> lower_limits = {-0.400924, 1.23538, -2.8973, -3.0718,
                                        -2.8973,   -0.0175, -2.8973};
  std::array<double, 7> upper_limits = {-0.400924, 1.23538, 2.8973, -0.0698,
                                        2.8973,    3.7525,  2.8973};

  // set limits
  KDL::JntArray ll, ul;
  ll.resize(chain.getNrOfJoints());
  ul.resize(chain.getNrOfJoints());
  for (int i = 0; i < chain.getNrOfJoints(); i++) {
    ll(i) = lower_limits[i];
    ul(i) = upper_limits[i];
  }

  // create ik solver
  TRAC_IK::TRAC_IK tracik_solver(chain, ll, ul);
  // create ik problem
  KDL::JntArray q(7);
  KDL::Frame frame;
  frame.p.x(pose[0]);
  frame.p.y(pose[1]);
  frame.p.z(pose[2]);
  frame.M = KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  KDL::JntArray q_init(7);
  for (int i = 0; i < 7; i++) {
    q_init(i) = 0;
  }

  // solve ik
  int rc = tracik_solver.CartToJnt(q_init, frame, q);
  if (rc < 0) {
    std::cout << "Failed to solve ik" << std::endl;
    success = false;
  }

  // return joint angles
  std::array<double, 7> joint_angles;
  for (int i = 0; i < 7; i++) {
    joint_angles[i] = q(i);
  }

  s.joint_angles = joint_angles;
  s.success = success;

  return s;
}

// ik using trac_ik
successJointAngles PandaInterface::ik(std::array<double, 7> pose,
                                      KDL::Tree tree) {
  /*
   * Get the joint angles for a given pose using trac_ik
   * @param pose: 7 element array of doubles representing the pose of the end
   * effector in the form [x, y, z, qx, qy, qz, qw]
   * @param urdf_path: path to urdf file
   * @return: a struct containing a bool representing whether the ik was
   * successful and a 7 element array of doubles representing the joint angles
   */
  // load robot model from urdf
  std::string name = "panda";
  successJointAngles s;
  bool success = true;

  // create tree from urdf_string
  // KDL::Tree tree;
  // kdl_parser::treeFromFile(URDF_PATH, tree);

  // create chain from tree
  KDL::Chain chain;
  tree.getChain(name + "_link0", name + "_hand", chain);
  // std::cout << "Num Joints in chain: " << chain.getNrOfJoints() << std::endl;

  std::array<double, 7> lower_limits = {-2.8973, -1.7628, -2.8973, -3.0718,
                                        -2.8973, -0.0175, -2.8973};
  std::array<double, 7> upper_limits = {2.8973, 1.7628, 2.8973, -0.0698,
                                        2.8973, 3.7525, 2.8973};

  // set limits
  KDL::JntArray ll, ul;
  ll.resize(chain.getNrOfJoints());
  ul.resize(chain.getNrOfJoints());
  for (int i = 0; i < chain.getNrOfJoints(); i++) {
    ll(i) = lower_limits[i];
    ul(i) = upper_limits[i];
  }

  // create ik solver
  TRAC_IK::TRAC_IK tracik_solver(chain, ll, ul);
  ;

  // create ik problem
  KDL::JntArray q(7);
  KDL::Frame frame;
  frame.p.x(pose[0]);
  frame.p.y(pose[1]);
  frame.p.z(pose[2]);
  frame.M = KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  KDL::JntArray q_init(7);
  for (int i = 0; i < 7; i++) {
    q_init(i) = 0;
  }

  // solve ik
  int rc = tracik_solver.CartToJnt(q_init, frame, q);
  if (rc < 0) {
    std::cout << "Failed to solve ik" << std::endl;
    success = false;
  }

  // return joint angles
  std::array<double, 7> joint_angles;
  for (int i = 0; i < 7; i++) {
    joint_angles[i] = q(i);
  }

  s.joint_angles = joint_angles;
  s.success = success;

  return s;
}

// fk solver using kdl
std::array<double, 7> PandaInterface::fk(std::array<double, 7> q) {
  /*
   * Calculate End Effector Pose from Joint Angles using KDL
   * @param q: joint angles
   * @param urdf_path: path to urdf file
   * @return: end effector pose
   */
  // instantiate kdl tree
  KDL::Tree my_tree;
  kdl_parser::treeFromFile(URDF_PATH, my_tree);
  KDL::Chain chain;
  // get chain from base to end effector
  my_tree.getChain("panda_link0", "panda_hand", chain);
  // set joint positions
  KDL::JntArray jointpositions = KDL::JntArray(chain.getNrOfJoints());
  for (int i = 0; i < 7; i++) {
    jointpositions(i) = q[i];
  }
  // instantiate frame
  KDL::Frame cartpos;
  KDL::ChainFkSolverPos_recursive fksolver =
      KDL::ChainFkSolverPos_recursive(chain);
  bool kinematics_status;
  // calculate forward kinematics
  kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  // return end effector pose if successful
  if (kinematics_status >= 0) {
    std::array<double, 7> ee_pose;
    for (int i = 0; i < 3; i++) {
      ee_pose[i] = cartpos.p[i];
    }
    for (int i = 0; i < 4; i++) {
      ee_pose[i + 3] = cartpos.M.data[i];
    }
    return ee_pose;
  } else {
    std::cout << "Error: could not calculate forward kinematics" << std::endl;
    return q;
    // TODO: throw error
  }
}

void PandaInterface::follow_joint_waypoints(
    franka::Robot *robot, std::vector<std::array<double, 7>> joint_waypoints,
    double speed_factor) {
  move_to_joint_angles(robot, joint_waypoints[0]);
  for (int i = 1; i < joint_waypoints.size(); i++) {
    move(robot, joint_waypoints[i - 1], joint_waypoints[i]);
  }
}

void PandaInterface::follow_joint_velocities(
    franka::Robot *robot, std::vector<std::array<double, 7>> joint_velocities) {
  //    move_to_joint_angles(robot, [0]);
  move_with_velocity_control(robot, joint_velocities);
}

std::tuple<ruckig::Trajectory<7>, bool> PandaInterface::generate_trajectory(
    ruckig::InputParameter<7> input) {
  ruckig::OutputParameter<7> output;  // Number DoFs
  ruckig::Trajectory<7> trajectory;
  ruckig::Result result = ruckig_7.calculate(input, trajectory);

  bool success = true;
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Ruckig Trajectory::Invalid input!" << std::endl;
    success = false;
  }

  return std::make_tuple(trajectory, success);
}
std::tuple<ruckig::Trajectory<6>, bool>
PandaInterface::generate_trajectory_cartesian(ruckig::InputParameter<6> input) {
  ruckig::OutputParameter<6> output;  // Number DoFs
  ruckig::Trajectory<6> trajectory;
  ruckig::Result result = ruckig_6.calculate(input, trajectory);

  bool success = true;
  if (result == ruckig::Result::ErrorInvalidInput) {
    std::cout << "Ruckig Trajectory::Invalid input!" << std::endl;
    success = false;
  }

  return std::make_tuple(trajectory, success);
}

bool PandaInterface::execute_trajectory_cartesian(
    franka::Robot *robot, ruckig::Trajectory<6> trajectory) {
  double duration = trajectory.get_duration();
  double time = 0;
  bool motion_finished = false;
  std::array<double, 6> new_position;
  std::array<double, 6> new_velocity;
  std::array<double, 6> new_acceleration;

  auto cartesian_velocity_call_back =
      [&](const franka::RobotState &robot_state,
          franka::Duration period) -> franka::CartesianVelocities {
    time += period.toSec();
    if (time >= duration && motion_finished) {
      return franka::MotionFinished(
          franka::CartesianVelocities({0, 0, 0, 0, 0, 0}));
    } else if (time >= duration && !motion_finished) {
      franka::CartesianVelocities output_velocities = {0, 0, 0, 0, 0, 0};
      motion_finished = true;
      return output_velocities;
    } else {
      trajectory.at_time(time, new_position, new_velocity, new_acceleration);
      franka::CartesianVelocities output_velocities = new_velocity;
      return output_velocities;
    }
  };
  robot->control(cartesian_velocity_call_back);
  return true;
}

bool PandaInterface::move_cartesian(franka::Robot *robot,
                                    std::array<double, 6> current_pose,
                                    std::array<double, 6> target_pose) {
  ROS_INFO("Moving to target cartesian pose");
  std::array<double, 6> initial_pose_dot = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> initial_pose_ddot = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> target_pose_dot = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> target_pose_ddot = {0, 0, 0, 0, 0, 0};
  ruckig::InputParameter input_parameter = get_ruckig_input_cartesian(
      current_pose, initial_pose_dot, initial_pose_ddot, target_pose,
      target_pose_dot, target_pose_ddot);
  std::tuple<ruckig::Trajectory<6>, bool> trajectory =
      generate_trajectory_cartesian(input_parameter);
  bool success = std::get<1>(trajectory);
  if (success) {
    ruckig::Trajectory<6> traj = std::get<0>(trajectory);
    success = execute_trajectory_cartesian(robot, traj);
  }
  return success;
}

void PandaInterface::move_failure(
    franka::Robot *robot, ruckig::Trajectory<5> *trajectory, int failure_mode,
    std::tuple<std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<std::array<double, 7>, NUM_STEPS>,
               std::array<double, NUM_STEPS>, std::array<double, 2>, int>
        *trajectory_data) {
  try {
    double duration = trajectory->get_duration();
    std::cout << "Duration: " << duration << std::endl;
    double fuck_time = 0;
    bool motion_finished = false;
    std::array<double, 5> new_position;
    std::array<double, 5> new_velocity;
    std::array<double, 5> new_acceleration;

    // int NUM_STEPS = int(duration*1000);
    std::array<std::array<double, 7>, NUM_STEPS> joint_angles;
    std::array<std::array<double, 7>, NUM_STEPS> joint_velocities;
    std::array<std::array<double, 7>, NUM_STEPS> joint_accelerations;
    std::array<double, NUM_STEPS> timestamps;
    std::array<double, 2> edge_time_stamps = {0.0, duration};

    int idx = 0;

    double last_time = 0;

    auto joint_velocity_call_back =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::JointVelocities {
      fuck_time += period.toSec();

      std::cout << "Time Left: " << duration - fuck_time
                << ", period to sec: " << period.toSec() << std::endl;

      // if (fuck_time < 0.3){
      //     std::cout << "Time went backwards" << std::endl;
      // }
      std::cout << "----" << std::endl;
      // if (period.toSec() < 0.001) {
      //     std::cout <<" period.toSec() < 0.001 "<<period.toSec()<<std::endl;
      // }
      last_time = fuck_time;
      timestamps[idx] = fuck_time;
      joint_angles[idx] = robot_state.q;
      joint_velocities[idx] = robot_state.dq;
      joint_accelerations[idx] = robot_state.ddq_d;

      idx++;

      // if (fuck_time >= duration && motion_finished){
      //     std::cout << "Motion finished2" << std::endl;
      //     return franka::MotionFinished(franka::JointVelocities({0, 0, 0, 0,
      //     0, 0, 0}));
      // }
      if (fuck_time >= duration && !motion_finished) {
        std::cout << "Motion finished1" << std::endl;
        franka::JointVelocities output_velocities = {0, 0, 0, 0, 0, 0, 0};
        if (allCloseZero(robot_state.dq, 0.03)) {
          motion_finished = true;
          return franka::MotionFinished(
              franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
        }

        return output_velocities;
      } else {
        trajectory->at_time(fuck_time, new_position, new_velocity,
                            new_acceleration);
        std::cout << "in main motion loop**************************************"
                  << std::endl;
        std::array<double, 7> new_velocity_7;
        switch (failure_mode) {
          case 1:
            new_velocity_7 = {0,
                              0,
                              new_velocity[0],
                              new_velocity[1],
                              new_velocity[2],
                              new_velocity[3],
                              new_velocity[4]};
            break;
          case 2:
            new_velocity_7 = {new_velocity[0],
                              new_velocity[1],
                              new_velocity[2],
                              new_velocity[3],
                              new_velocity[4],
                              0,
                              0};
            break;
        }

        // std::cout << "Time left: " << duration - time << std::endl;
        // std::cout << "Time: " << time << std::endl;
        // std::cout<<"new velocity 7: "<<new_velocity_7[0]<<",
        // "<<new_velocity_7[1]<<", "<<new_velocity_7[2]<<",
        // "<<new_velocity_7[3]<<", "<<new_velocity_7[4]<<",
        // "<<new_velocity_7[5]<<", "<<new_velocity_7[6]<<std::endl;
        franka::JointVelocities output_velocities = new_velocity_7;
        return output_velocities;
      }
    };
    robot->control(joint_velocity_call_back);

    std::cout << "gg ixd: " << idx << std::endl;
    std::cout << "bytes of joint angles: " << sizeof(joint_angles) << std::endl;
    std::cout << "bytes of joint velocities: " << sizeof(joint_velocities)
              << std::endl;
    std::cout << "bytes of joint accelerations: " << sizeof(joint_accelerations)
              << std::endl;
    std::cout << "bytes of timestamps: " << sizeof(timestamps) << std::endl;
    std::cout << "bytes of edge timestamps: " << sizeof(edge_time_stamps)
              << std::endl;

    *trajectory_data =
        std::make_tuple(joint_angles, joint_velocities, joint_accelerations,
                        timestamps, edge_time_stamps, idx);
  } catch (const std::exception &ex) {
    // std::exception_ptr p = std::current_exception();

    std::cout << ex.what() << std::endl;
    robot->automaticErrorRecovery();
    *trajectory_data = std::make_tuple(
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<double, NUM_STEPS>(), std::array<double, 2>(), 0);
  } catch (const franka::ControlException &err) {
    std::cout << "Control Exception" << std::endl;
    robot->automaticErrorRecovery();
    *trajectory_data = std::make_tuple(
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<double, NUM_STEPS>(), std::array<double, 2>(), 0);
  } catch (const franka::Exception &er) {
    std::cout << er.what() << std::endl;
    robot->automaticErrorRecovery();
    *trajectory_data = std::make_tuple(
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<std::array<double, 7>, NUM_STEPS>(),
        std::array<double, NUM_STEPS>(), std::array<double, 2>(), 0);
  }
}

bool PandaInterface::edges_to_npy(
    std::array<
        std::tuple<
            std::array<std::array<double, 7>, NUM_STEPS>,  // position
            std::array<std::array<double, 7>, NUM_STEPS>,  // velocity
            std::array<std::array<double, 7>, NUM_STEPS>,  // acceleration
            std::array<double, NUM_STEPS>,                 // timestap
            std::array<double, 2>,                         // edge time stamps
            int>,
        NUM_EDGES>
        edges) {
  //     # q_trajs.extend(planar_eb_misc['q'])
  // # qddot_trajs

  std::array<std::array<double, NUM_STEPS>, NUM_EDGES> t;
  std::array<std::array<double, 2>, NUM_EDGES> edge_timestamp_range;
  std::array<std::array<double, 7>, NUM_EDGES> traj_start_q;
  std::array<std::array<std::array<double, 7>, NUM_STEPS>, NUM_EDGES> qdot;
  std::array<std::array<std::array<double, 7>, NUM_STEPS>, NUM_EDGES> qddot;
  std::array<std::array<std::array<double, 7>, NUM_STEPS>, NUM_EDGES> qtrajs;
  std::array<int, NUM_EDGES> num_steps;

  for (int i = 0; i < NUM_EDGES; i++) {
    std::array<double, NUM_STEPS> timestamps = std::get<3>(edges[i]);
    std::array<std::array<double, 7>, NUM_STEPS> joint_angles =
        std::get<0>(edges[i]);
    std::array<std::array<double, 7>, NUM_STEPS> joint_velocities =
        std::get<1>(edges[i]);
    std::array<std::array<double, 7>, NUM_STEPS> joint_acc =
        std::get<2>(edges[i]);

    num_steps[i] = std::get<5>(edges[i]);
    edge_timestamp_range[i] = std::get<4>(edges[i]);
    traj_start_q[i] = joint_angles[0];

    t[i] = timestamps;
    qdot[i] = joint_velocities;
    qtrajs[i] = joint_angles;
    qddot[i] = joint_acc;
  }

  auto t_ = std::time(nullptr);
  auto tm = *std::localtime(&t_);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
  auto str = oss.str();

  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "num_steps", &num_steps[0],
  //                {NUM_EDGES}, "w");
  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "edge_timestamp_range",
  //                &edge_timestamp_range[0][0], {NUM_EDGES, 2}, "a");
  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "traj_start_q",
  //                &traj_start_q[0][0], {NUM_EDGES, 7}, "a");
  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "t", &t[0][0],
  //                {NUM_EDGES, NUM_STEPS}, "a");
  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "qdot", &qdot[0][0][0],
  //                {NUM_EDGES, NUM_STEPS, 7}, "a");

  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "qddot", &qddot[0][0][0],
  //                {NUM_EDGES, NUM_STEPS, 7}, "a");
  // cnpy::npz_save("eb_by_hand_" + str + ".npz", "q", &qtrajs[0][0][0],
  //                {NUM_EDGES, NUM_STEPS, 7}, "a");
  // TODO: make it return false if the file is not saved
  return true;
}

bool PandaInterface::move(franka::Robot *robot,
                          std::array<double, 7> current_joint_angles,
                          std::array<double, 7> target_joint_angles) {
  ROS_INFO("Moving to target joint angles");
  std::array<double, 7> initial_q_dot = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> initial_q_ddot = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> target_q_dot = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> target_q_ddot = {0, 0, 0, 0, 0, 0, 0};
  ruckig::InputParameter input_parameter =
      get_ruckig_input(current_joint_angles, initial_q_dot, initial_q_ddot,
                       target_joint_angles, target_q_dot, target_q_ddot);
  std::tuple<ruckig::Trajectory<7>, bool> trajectory_succ_tuple =
      generate_trajectory(input_parameter);
  ruckig::Trajectory<7> trajectory = std::get<0>(trajectory_succ_tuple);
  bool success = std::get<1>(trajectory_succ_tuple);
  if (!success) {
    std::cout << "Ruckig Trajectory::Failed to generate trajectory"
              << std::endl;
    return false;
  }
  double duration = trajectory.get_duration() + 0.01;
  double time = 0;
  bool motion_finished = false;
  std::array<double, 7> new_position;
  std::array<double, 7> new_velocity;
  std::array<double, 7> new_acceleration;

  auto joint_velocity_call_back =
      [&](const franka::RobotState &robot_state,
          franka::Duration period) -> franka::JointVelocities {
    time += period.toSec();
    if (time >= duration && motion_finished) {
      return franka::MotionFinished(
          franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
    } else if (time >= duration && !motion_finished) {
      franka::JointVelocities output_velocities = {0, 0, 0, 0, 0, 0, 0};
      motion_finished = true;
      return output_velocities;
    } else {
      trajectory.at_time(time, new_position, new_velocity, new_acceleration);
      franka::JointVelocities output_velocities = new_velocity;
      return output_velocities;
    }
  };
  robot->control(joint_velocity_call_back);
  return true;
}

bool PandaInterface::allCloseZero(const std::array<double, 7> &arr,
                                  double tolerance) {
  auto withinTolerance = [&](double x) { return std::abs(x) <= tolerance; };
  return std::all_of(arr.begin(), arr.end(), withinTolerance);
}

bool PandaInterface::move_with_velocity_control(
    franka::Robot *robot, std::vector<std::array<double, 7>> joint_velocities) {
  double duration = double(joint_velocities.size()) * 0.001 - 0.002;
  double time = 0;
  bool motion_finished = false;
  bool done = false;
  int i = 0;
  while (!done) {
    std::array<double, 7> new_velocity = joint_velocities[i];

    auto joint_velocity_call_back =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::JointVelocities {
      std::cout << "period: " << period.toSec() << std::endl;
      time += 0.001;
      std::array<double, 7> new_velocity = joint_velocities[i];
      i++;

      if (time >= duration && motion_finished) {
        done = true;
        return franka::MotionFinished(
            franka::JointVelocities({0, 0, 0, 0, 0, 0, 0}));
      } else if (time >= duration && !motion_finished) {
        franka::JointVelocities output_velocities = {0, 0, 0, 0, 0, 0, 0};
        std::array<double, 7> dq = robot_state.dq;
        if (allCloseZero(dq, 0.01)) {
          motion_finished = true;
        }
        return output_velocities;
      } else {
        franka::JointVelocities output_velocities = new_velocity;

        std::cout << std::endl;
        for (auto vel : new_velocity) {
          std::cout << vel << ", ";
        }
        std::cout << std::endl;

        return output_velocities;
      }
    };
    robot->control(joint_velocity_call_back);
  }

  return true;
}

std::array<double, 3> PandaInterface::quaternion_to_euler(double x, double y,
                                                          double z, double w) {
  std::array<double, 3> euler;
  double ysqr = y * y;
  double t0 = -2.0 * (ysqr + z * z) + 1.0;
  double t1 = +2.0 * (x * y - w * z);
  double t2 = -2.0 * (x * z + w * y);
  double t3 = +2.0 * (y * z - w * x);
  double t4 = -2.0 * (x * x + ysqr) + 1.0;
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  euler[0] = std::atan2(t3, t4);
  euler[1] = std::asin(t2);
  euler[2] = std::atan2(t1, t0);
  return euler;
}

void PandaInterface::move_down_and_interact(franka::Robot *robot,
                                            franka::Gripper *gripper,
                                            float height, double object_width,
                                            bool interaction) {
  double speed = 0.1;
  double force = 60;
  std::array<double, 7> current_joint_angles = robot->readOnce().q;
  std::array<double, 7> current_pose = fk(current_joint_angles);
  std::array<double, 7> goal_pose = current_pose;

  goal_pose[2] = goal_pose[2] - height;

  // convert current pose and goal pose quaternions to euler angles
  std::array<double, 3> current_pose_euler = quaternion_to_euler(
      current_pose[3], current_pose[4], current_pose[5], current_pose[6]);
  std::array<double, 3> goal_pose_euler = quaternion_to_euler(
      goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]);

  std::cout << "Current Pose: " << current_pose[0] << ", " << current_pose[1]
            << ", " << current_pose[2] << ", " << current_pose_euler[0] << ", "
            << current_pose_euler[1] << ", " << current_pose_euler[2]
            << std::endl;
  std::cout << "Goal Pose: " << goal_pose[0] << ", " << goal_pose[1] << ", "
            << goal_pose[2] << ", " << goal_pose_euler[0] << ", "
            << goal_pose_euler[1] << ", " << goal_pose_euler[2] << std::endl;

  // set goal pose euler angles to current pose euler angles
  std::array<double, 6> goal_pose_euler_array = {
      goal_pose[0],       goal_pose[1],       goal_pose[2],
      goal_pose_euler[0], goal_pose_euler[1], goal_pose_euler[2]};
  std::array<double, 6> current_pose_euler_array = {
      current_pose[0],       current_pose[1],       current_pose[2],
      current_pose_euler[0], current_pose_euler[1], current_pose_euler[2]};

  // ROS_INFO("Getting joint Angles");
  successJointAngles target = ik(goal_pose);
  // ROS_INFO("Opening Gripper");
  // gripper->homing();
  if (interaction) {  // open gripper
    // open gripper
    franka::GripperState gripper_state = gripper->readOnce();
    gripper->move(gripper_state.max_width, speed);
  }
  ROS_INFO("Moving Down");
  bool up_success = false;
  if (target.success) {
    ROS_INFO("Moving Down");
    // up_success = move(robot, current_joint_angles, target.joint_angles);
    up_success =
        move_cartesian(robot, current_pose_euler_array, goal_pose_euler_array);
  } else {
    std::cout << "Failed to move down" << std::endl;
  }
  if (!up_success) {
    std::cout << "Failed to move down" << std::endl;
  }

  current_joint_angles = robot->readOnce().q;
  current_pose = fk(current_joint_angles);
  current_pose_euler = quaternion_to_euler(current_pose[3], current_pose[4],
                                           current_pose[5], current_pose[6]);
  std::cout << "Current Pose Final: " << current_pose[0] << ", "
            << current_pose[1] << ", " << current_pose[2] << ", "
            << current_pose_euler[0] << ", " << current_pose_euler[1] << ", "
            << current_pose_euler[2] << std::endl;

  if (interaction) {  // pick up object
    ROS_INFO("Grasping");
    gripper->grasp(object_width, speed, force, 0.02, 0.02);
  } else {  // place object
    gripper->stop();
    franka::GripperState gripper_state = gripper->readOnce();
    gripper->move(gripper_state.max_width, speed);
    gripper->stop();
  }
  ROS_INFO("Moving Up");
  // bool down_success = move(robot, target.joint_angles, current_joint_angles);
  bool down_success =
      move_cartesian(robot, goal_pose_euler_array, current_pose_euler_array);
}

void PandaInterface::cleanup(franka::Robot *robot, franka::Gripper *gripper) {
  delete &ruckig_5;
  delete &ruckig_6;
  delete &ruckig_7;
  delete robot;
  delete gripper;
}
