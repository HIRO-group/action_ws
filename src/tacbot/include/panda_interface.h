#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <ruckig/ruckig.hpp>

// #include "panda_sim_real_interface/JointDataArray.h"
// #include <fstream>

// #include <ruckig/ruckig.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

#ifndef SUCCESS_JOINT_ANGLES_H
#define SUCCESS_JOINT_ANGLES_H
struct successJointAngles {
  std::array<double, 7> joint_angles;
  bool success;
};
#endif

#define NUM_STEPS 2000
#define NUM_EDGES 1

class PandaInterface {
 public:
  std::shared_ptr<franka::Robot> robot_;
  std::shared_ptr<franka::Model> robot_model_;
  std::string franka_address_ = "192.168.1.178";

  void init();

  std::array<double, 7> fk(std::array<double, 7> q);
  successJointAngles ik(std::array<double, 7> ee_pose, KDL::Tree tree);
  successJointAngles ik(std::array<double, 7> ee_pose);
  void set_joint_and_collision_behaviour(franka::Robot *robot);
  void move_to_default_pose(franka::Robot *robot);
  void move_to_joint_angles(franka::Robot *robot, std::array<double, 7> q_goal);
  int get_closest_time(float time, std::vector<double> time_list);
  void follow_joint_waypoints(
      franka::Robot *robot, std::vector<std::array<double, 7>> joint_waypoints,
      double speed_factor);
  void move_down_and_interact(franka::Robot *robot, franka::Gripper *gripper,
                              float height, double object_width,
                              bool interaction);
  bool move(franka::Robot *robot, std::array<double, 7> current_joint_angles,
            std::array<double, 7> target_joint_angles);
  std::tuple<ruckig::Trajectory<7>, bool> generate_trajectory(
      ruckig::InputParameter<7> input);
  ruckig::InputParameter<7> get_ruckig_input(
      std::array<double, 7> inital_q, std::array<double, 7> inital_q_vel,
      std::array<double, 7> inital_q_acc, std::array<double, 7> target_q,
      std::array<double, 7> target_q_vel, std::array<double, 7> target_q_acc);
  bool move_cartesian(franka::Robot *robot, std::array<double, 6> current_pose,
                      std::array<double, 6> target_pose);
  bool execute_trajectory_cartesian(franka::Robot *robot,
                                    ruckig::Trajectory<6> trajectory);
  std::array<double, 3> quaternion_to_euler(double x, double y, double z,
                                            double w);
  ruckig::InputParameter<6> get_ruckig_input_cartesian(
      std::array<double, 6> inital_q, std::array<double, 6> inital_q_vel,
      std::array<double, 6> inital_q_acc, std::array<double, 6> target_q,
      std::array<double, 6> target_q_vel, std::array<double, 6> target_q_acc);
  std::tuple<ruckig::Trajectory<6>, bool> generate_trajectory_cartesian(
      ruckig::InputParameter<6> input);
  bool move_with_velocity_control(
      franka::Robot *robot,
      std::vector<std::array<double, 7>> joint_velocities);
  void follow_joint_velocities(
      franka::Robot *robot,
      std::vector<std::array<double, 7>> joint_velocities);
  bool allCloseZero(const std::array<double, 7> &arr, double tolerance);
  void move_failure(franka::Robot *robot, ruckig::Trajectory<5> *trajectory,
                    int failure_mode,
                    std::tuple<std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<double, NUM_STEPS>,
                               std::array<double, 2>, int> *trajectory_data);
  bool edges_to_npy(
      std::array<
          std::tuple<std::array<std::array<double, 7>, NUM_STEPS>,
                     std::array<std::array<double, 7>, NUM_STEPS>,
                     std::array<std::array<double, 7>, NUM_STEPS>,
                     std::array<double, NUM_STEPS>, std::array<double, 2>, int>,
          NUM_EDGES>
          edges);
  ruckig::InputParameter<5> get_ruckig_input_5(
      std::array<double, 5> inital_q, std::array<double, 5> inital_q_vel,
      std::array<double, 5> inital_q_acc, std::array<double, 5> target_q,
      std::array<double, 5> target_q_vel, std::array<double, 5> target_q_acc);
  ruckig::Trajectory<5> get_ruckig_trajectory_5(
      std::array<double, 7> initial_q_7, std::array<double, 7> target_q_7);
  void perform_edge(std::array<double, 7> initial_q,
                    std::array<double, 7> target_q, franka::Robot *robot,
                    std::tuple<std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<std::array<double, 7>, NUM_STEPS>,
                               std::array<double, NUM_STEPS>,
                               std::array<double, 2>, int> *trajectory_data);
  void cleanup(franka::Robot *robot, franka::Gripper *gripper);
};