#include "contact_planner.h"

#include <math.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <chrono>

using namespace std::chrono;
constexpr char LOGNAME[] = "contact_planner";

namespace tacbot {

ContactPlanner::ContactPlanner() : BasePlanner() {
  setObstacleScene(1);
  setGoalState(1);
  contact_perception_ = std::make_shared<ContactPerception>();
  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  contact_perception_->init();
  extractPtsFromGoalState();
}

void ContactPlanner::setGoalState(std::size_t option) {
  switch (option) {
    default:
    case 1:
      joint_goal_pos_ =
          std::vector<double>{-1.0, 0.7, 0.7, -1.0, -0.7, 2.0, 0.0};
      break;
  }
}

void ContactPlanner::setObstacleScene(std::size_t option) {
  spherical_obstacles_.clear();
  sim_obstacle_pos_.clear();
  ROS_INFO_NAMED(LOGNAME, "Obstacle scene option selected: %ld", option);
  switch (option) {
    case 1:
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.5, 0.0, 0.6}, 0.1));
      break;
    case 2:
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.45, -0.4, 0.6}, 0.1));
      break;
    case 3:
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.4, 0.0, 0.6}, 0.1));
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.5, -0.5, 0.5}, 0.1));
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{-0.1, -0.15, 0.6}, 0.1));
      break;
    case 4:
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.40, 0.0, 0.6}, 0.1));
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{-0.2, 0.0, 0.5}, 0.1));
      spherical_obstacles_.emplace_back(
          std::make_pair(Eigen::Vector3d{0.15, 0.2, 0.7}, 0.1));
      break;
    default:
      ROS_ERROR_NAMED(LOGNAME,
                      "Selected obstacle scene option is out of bounds: %ld",
                      option);
      break;
  }

  for (auto obstacle : spherical_obstacles_) {
    addSphericalObstacle(obstacle.first, obstacle.second);
  }
}

void ContactPlanner::addSphericalObstacle(const Eigen::Vector3d& center,
                                          double radius) {
  const double inc = 0.4;
  for (double theta = 0; theta < 2 * M_PI; theta += inc) {
    for (double phi = 0; phi < 2 * M_PI; phi += inc) {
      double x = radius * cos(phi) * sin(theta) + center[0];
      double y = radius * sin(phi) * sin(theta) + center[1];
      double z = radius * cos(theta) + center[2];
      sim_obstacle_pos_.emplace_back(Eigen::Vector3d{x, y, z});
    }
  }
}

Eigen::Vector3d ContactPlanner::scaleToDist(Eigen::Vector3d vec) {
  double y_max = 3.0;
  double D = 50.0;
  // double dist_max = 1.0;
  // std::cout << "vec.norm() " << vec.norm() << std::endl;
  Eigen::Vector3d vec_out = Eigen::VectorXd::Zero(3);

  // double norm = vec.norm();
  // double norm_max = 0.1;
  // if (norm < norm_max) {
  //   vec_out[0] = 1;
  //   vec_out[1] = 1;
  //   vec_out[2] = 1;
  // } else {
  //   vec_out[0] = 0;
  //   vec_out[1] = 0;
  //   vec_out[2] = 0;
  // }

  double prox_radius = 0.02;
  if (planner_id_ == "ContactTRRTDuo" || planner_id_ == "VFRRT") {
    prox_radius = 0.2;
  }

  if (vec.squaredNorm() > prox_radius) {
    return vec_out;
  }

  if (planner_id_ != "ContactTRRTDuo") {
    vec.normalize();
    return vec;
  }

  vec_out = (vec * y_max) / (D * vec.squaredNorm() + 1.0);

  // std::cout << "vec.squaredNorm() " << vec.squaredNorm() << std::endl;
  // std::cout << "before scale " << vec.transpose() << std::endl;
  // std::cout << "vec_out " << vec_out.transpose() << std::endl;
  return vec_out;
}

void ContactPlanner::extractPtsFromGoalState() {
  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_goal_pos_);
  std::size_t num_pts = getPtsOnRobotSurface(robot_state, goal_rob_pts_);
  std::cout << "num pts from robot goal state: " << num_pts << std::endl;
}

Eigen::Vector3d ContactPlanner::getAttractPt(std::size_t link_num,
                                             std::size_t pt_num) {
  std::vector<Eigen::Vector3d> pts_on_link = goal_rob_pts_[link_num];
  Eigen::Vector3d pt_on_rob = pts_on_link[pt_num];
  return pt_on_rob;
}

void ContactPlanner::extractPtsFromModel(
    const moveit::core::RobotStatePtr& robot_state,
    const moveit::core::LinkModel* link_model,
    std::vector<Eigen::Vector3d>& link_pts, std::size_t& num_pts) {
  std::vector<shapes::ShapeConstPtr> shapes = link_model->getShapes();
  std::size_t num_shapes = shapes.size();
  for (std::size_t j = 0; j < num_shapes; ++j) {
    Eigen::Isometry3d transform =
        robot_state->getCollisionBodyTransform(link_model, j);

    if (shapes[j]->type != shapes::MESH) {
      // should somehow extract points from this as well
      // ROS_ERROR_NAMED(LOGNAME, "Not a mesh shape");
    } else {
      shapes::Mesh* mesh = static_cast<shapes::Mesh*>(shapes[j]->clone());
      mesh->mergeVertices(0.05);
      // std::vector<Eigen::Vector3d> points(mesh->vertex_count);

      // std::vector<Eigen::Vector3d> points(mesh->vertex_count);

      for (unsigned int k = 0; k < mesh->vertex_count; ++k) {
        Eigen::Vector3d mesh_pt(mesh->vertices[3 * k],
                                mesh->vertices[3 * k + 1],
                                mesh->vertices[3 * k + 2]);
        // points[k] = transform * mesh_pt;
        link_pts.emplace_back(transform * mesh_pt);
        num_pts++;
      }

      // for (std::size_t p = 0; p < mesh->triangle_count; ++p) {
      //   std::size_t i3 = p * 3;
      //   Eigen::Vector3d s1(mesh->vertices[mesh->triangles[i3] * 3],
      //                      mesh->vertices[mesh->triangles[i3] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3] * 3 + 2]);

      //   Eigen::Vector3d s2(mesh->vertices[mesh->triangles[i3 + 1] * 3],
      //                      mesh->vertices[mesh->triangles[i3 + 1] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3 + 1] * 3 +
      //                      2]);

      //   Eigen::Vector3d s3(mesh->vertices[mesh->triangles[i3 + 2] * 3],
      //                      mesh->vertices[mesh->triangles[i3 + 2] * 3 + 1],
      //                      mesh->vertices[mesh->triangles[i3 + 2] * 3 +
      //                      2]);
      //   points.emplace_back(transform * ((s1 + s2) / 2));
      //   points.emplace_back(transform * ((s2 + s3) / 2));
      //   points.emplace_back(transform * ((s3 + s1) / 2));
      //   num_pts = num_pts + 3;
      // }
    }
  }
}

std::size_t ContactPlanner::getPtsOnRobotSurface(
    const moveit::core::RobotStatePtr& robot_state,
    std::vector<std::vector<Eigen::Vector3d>>& rob_pts) {
  const std::vector<std::string> link_names =
      joint_model_group_->getLinkModelNames();

  rob_pts.clear();
  std::size_t num_links = link_names.size();
  std::size_t num_pts = 0;
  // std::cout << "num_links " << num_links << std::endl;

  for (std::size_t i = 0; i < num_links; i++) {
    // std::cout << "i: " << i << ", " << link_names[i] << std::endl;

    const moveit::core::LinkModel* link_model =
        robot_state->getLinkModel(link_names[i]);

    const moveit::core::LinkTransformMap fixed_links =
        link_model->getAssociatedFixedTransforms();

    // std::cout << "link_tf_map.size(): " << link_tf_map.size() << std::endl;

    std::vector<shapes::ShapeConstPtr> shapes = link_model->getShapes();
    std::size_t num_shapes = shapes.size();

    std::vector<Eigen::Vector3d> link_pts;

    // the last link does not have a mesh for some reason
    if (i >= dof_ - 1 || (num_shapes == 0 && fixed_links.size() > 0)) {
      // std::cout << "i: " << i << ", " << link_names[i] << std::endl;
      for (const std::pair<const moveit::core::LinkModel* const,
                           Eigen::Isometry3d>& fixed_link : fixed_links) {
        extractPtsFromModel(robot_state, fixed_link.first, link_pts, num_pts);
      }

      if (i == num_links - 1) {
        // std::cout << "i: " << i << ", " << link_names[i] << std::endl;
        rob_pts.emplace_back(link_pts);
      }
      continue;
    }

    extractPtsFromModel(robot_state, link_model, link_pts, num_pts);
    rob_pts.emplace_back(link_pts);
  }
  // std::cout << "rob_pts.size() " << rob_pts.size() << std::endl;
  return num_pts;
}

std::vector<Eigen::Vector3d> ContactPlanner::getObstacles(
    const Eigen::Vector3d& pt_on_rob) {
  std::vector<Eigen::Vector3d> obstacles;
  if (use_sim_obstacles_) {
    vis_data_->saveObstaclePos(sim_obstacle_pos_, sample_state_count_);
    return sim_obstacle_pos_;
  }

  bool status = contact_perception_->extractNearPts(pt_on_rob, obstacles);
  if (status) {
    vis_data_->saveObstaclePos(obstacles, sample_state_count_);
    return obstacles;
  }

  // Store an empty obstacle vector when no obstacles have been found in
  // proximity. This ensures that the size of the stored obstacles in an array
  // are equal to the number of states that we considered.
  vis_data_->saveObstaclePos(std::vector<Eigen::Vector3d>{},
                             sample_state_count_);

  // This yields a zero repulsion vector and will mean no repulsion will be
  // applied by the vectors filed.
  obstacles.emplace_back(pt_on_rob);
  return obstacles;
}

Eigen::VectorXd ContactPlanner::getRobtPtsVecDiffAvg(
    const std::vector<std::vector<Eigen::Vector3d>>& near_state_rob_pts,
    const std::vector<std::vector<Eigen::Vector3d>>& rand_state_rob_pts,
    const std::vector<Eigen::Vector3d>& link_to_obs_vec) {
  std::size_t num_links = near_state_rob_pts.size();
  Eigen::VectorXd mean_per_link = Eigen::VectorXd::Zero(num_links);

  std::size_t pt_num = 0;

  for (std::size_t i = 0; i < num_links; i++) {
    std::vector<Eigen::Vector3d> near_pts_on_link = near_state_rob_pts[i];
    std::vector<Eigen::Vector3d> rand_pts_on_link = rand_state_rob_pts[i];

    std::size_t num_pts_on_link = near_pts_on_link.size();
    Eigen::MatrixXd diff_per_link = Eigen::MatrixXd::Zero(num_pts_on_link, 3);

    for (std::size_t j = 0; j < num_pts_on_link; j++) {
      Eigen::Vector3d near_pt_on_rob = near_pts_on_link[j];
      Eigen::Vector3d rand_pt_on_rob = rand_pts_on_link[j];
      Eigen::Vector3d nearrand_diff = rand_pt_on_rob - near_pt_on_rob;
      nearrand_diff.normalize();
      nearrand_diff *= 0.05;

      diff_per_link(j, 0) = nearrand_diff[0];
      diff_per_link(j, 1) = nearrand_diff[1];
      diff_per_link(j, 2) = nearrand_diff[2];

      // std::cout << "pt_num: " << pt_num << std::endl;
      // std::cout << "sample_state_count_: " << sample_state_count_ <<
      // std::endl;

      vis_data_->saveNearRandVec(near_pt_on_rob, nearrand_diff, pt_num,
                                 sample_state_count_);

      pt_num++;
    }

    double av_x = diff_per_link.col(0).mean();
    double av_y = diff_per_link.col(1).mean();
    double av_z = diff_per_link.col(2).mean();

    Eigen::Vector3d nearrand_av(av_x, av_y, av_z);
    Eigen::Vector3d link_to_obs = link_to_obs_vec[i];

    std::cout << "nearrand_av: " << nearrand_av.transpose() << std::endl;
    std::cout << "link_to_obs: " << link_to_obs.transpose() << std::endl;
    double dot = link_to_obs.dot(nearrand_av);
    std::cout << "dot: " << dot << std::endl;

    mean_per_link[i] = dot;
  }
  vis_data_->saveNearRandDot(mean_per_link);
  return mean_per_link;
}

std::vector<Eigen::Vector3d> ContactPlanner::getLinkToObsVec(
    const std::vector<std::vector<Eigen::Vector3d>>& rob_pts) {
  std::size_t num_links = rob_pts.size();
  // std::cout << "num_links: " << num_links << std::endl;

  std::vector<Eigen::Vector3d> link_to_obs_vec(num_links,
                                               Eigen::Vector3d::Zero(3));

  std::size_t pt_num = 0;

  for (std::size_t i = 0; i < num_links; i++) {
    std::vector<Eigen::Vector3d> pts_on_link = rob_pts[i];
    std::size_t num_pts_on_link = pts_on_link.size();
    // std::cout << "link_num: " << i << std::endl;
    // std::cout << "num_pts_on_link: " << num_pts_on_link << std::endl;

    Eigen::MatrixXd pts_link_vec = Eigen::MatrixXd::Zero(num_pts_on_link, 3);

    std::vector<Eigen::Vector3d> obstacle_pos = getObstacles(pts_on_link[0]);

    for (std::size_t j = 0; j < num_pts_on_link; j++) {
      // std::cout << "pt j: " << j << std::endl;

      Eigen::Vector3d pt_on_rob = pts_on_link[j];
      // std::cout << "pt_on_rob: " << pt_on_rob.transpose() << std::endl;

      // std::vector<Eigen::Vector3d> obstacle_pos = getObstacles(pt_on_rob);

      std::size_t num_obstacles = obstacle_pos.size();
      // std::cout << "num_obstacles: " << num_obstacles << std::endl;

      Eigen::MatrixXd pt_to_obs = Eigen::MatrixXd::Zero(num_obstacles, 3);
      for (std::size_t k = 0; k < num_obstacles; k++) {
        Eigen::Vector3d vec = pt_on_rob - obstacle_pos[k];
        // std::cout << "vec: " << vec.transpose() << std::endl;
        // std::cout << "obstacle_pos: " << obstacle_pos[k].transpose()
        //           << std::endl;
        // std::cout << "pt_on_rob: " << pt_on_rob.transpose() << std::endl;

        vec = scaleToDist(vec);

        if (planner_id_ == "ContactTRRTDuo") {
          Eigen::Vector3d att_pt = getAttractPt(i, j);

          Eigen::Vector3d att_vec = att_pt - pt_on_rob;

          if (vec.norm() == 0.0) {
            att_vec = Eigen::VectorXd::Zero(3);
          } else {
            att_vec.normalize();
          }

          double dot = att_vec.dot(vec);
          if (dot < -0.5) {
            att_vec = Eigen::VectorXd::Zero(3);
          }

          vec = vec + 0.05 * att_vec;
          // std::cout << "vec: " << vec.transpose() << std::endl;
        }

        pt_to_obs(k, 0) = vec[0];
        pt_to_obs(k, 1) = vec[1];
        pt_to_obs(k, 2) = vec[2];
      }

      double av_x = pt_to_obs.col(0).mean();
      double av_y = pt_to_obs.col(1).mean();
      double av_z = pt_to_obs.col(2).mean();

      Eigen::Vector3d pt_to_obs_av(av_x, av_y, av_z);
      // std::cout << "pt_to_obs_av: " << pt_to_obs_av.transpose() << std::endl;

      pts_link_vec(j, 0) = pt_to_obs_av[0];
      pts_link_vec(j, 1) = pt_to_obs_av[1];
      pts_link_vec(j, 2) = pt_to_obs_av[2];

      // std::cout << "link1_idx: " << link1_idx << std::endl;
      // std::cout << "stop_crit: " << stop_crit << std::endl;
      // std::cout << "pt_on_rob: " << pt_on_rob.transpose() << std::endl;

      vis_data_->saveOriginVec(pt_on_rob, pt_to_obs_av, pt_num,
                               sample_state_count_);
      pt_num++;
    }
    // std::cout << "pts_link_vec.rows(): " << pts_link_vec.rows() << std::endl;
    double av_x = pts_link_vec.col(0).mean();
    double av_y = pts_link_vec.col(1).mean();
    double av_z = pts_link_vec.col(2).mean();
    Eigen::Vector3d link_to_obs_avg(av_x, av_y, av_z);
    // std::cout << "link_to_obs_avg: " << link_to_obs_avg.transpose()
    //           << std::endl;

    link_to_obs_vec[i] = link_to_obs_avg;
  }
  vis_data_->saveAvgRepulseVec(link_to_obs_vec);

  return link_to_obs_vec;
}

Eigen::VectorXd ContactPlanner::obstacleFieldTaskSpace(
    const ompl::base::State* base_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);

  std::vector<std::vector<Eigen::Vector3d>> rob_pts;
  std::size_t num_pts = getPtsOnRobotSurface(robot_state, rob_pts);
  vis_data_->setTotalNumRepulsePts(num_pts);
  std::vector<Eigen::Vector3d> link_to_obs_vec = getLinkToObsVec(rob_pts);
  Eigen::VectorXd field_out = Eigen::VectorXd::Zero(dof_);
  for (std::size_t i = 0; i < dof_; i++) {
    {
      Eigen::Vector3d vec = link_to_obs_vec[i];
      field_out[i] = vec.norm();
      // std::cout << "field_out[i]: " << field_out[i] << std::endl;
    }
  }
  return field_out;
}

Eigen::VectorXd ContactPlanner::obstacleFieldCartesian(
    const ompl::base::State* near_state, const ompl::base::State* rand_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state1 =
      *near_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles1 = utilities::toStlVec(vec_state1, dof_);

  moveit::core::RobotStatePtr robot_state1 =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state1->setJointGroupPositions(joint_model_group_, joint_angles1);
  std::vector<std::vector<Eigen::Vector3d>> near_rob_pts;
  std::size_t num_pts = getPtsOnRobotSurface(robot_state1, near_rob_pts);

  vis_data_->setTotalNumRepulsePts(num_pts);
  // std::cout << "num pts from near state: " << num_pts << std::endl;
  std::vector<Eigen::Vector3d> link_to_obs_vec = getLinkToObsVec(near_rob_pts);
  // std::cout << "link_to_obs_vec.size(): " << link_to_obs_vec.size()
  //           << std::endl;

  const ompl::base::RealVectorStateSpace::StateType& vec_state2 =
      *rand_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles2 = utilities::toStlVec(vec_state2, dof_);

  moveit::core::RobotStatePtr robot_state2 =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state2->setJointGroupPositions(joint_model_group_, joint_angles2);
  std::vector<std::vector<Eigen::Vector3d>> rand_rob_pts;
  num_pts = getPtsOnRobotSurface(robot_state2, rand_rob_pts);
  // std::cout << "num pts from rand state: " << num_pts << std::endl;

  Eigen::VectorXd vfield =
      getRobtPtsVecDiffAvg(near_rob_pts, rand_rob_pts, link_to_obs_vec);

  vis_data_->saveRepulseAngles(joint_angles1, joint_angles2);
  sample_state_count_++;
  return vfield;
}

Eigen::VectorXd ContactPlanner::obstacleFieldConfigSpace(
    const ompl::base::State* base_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);

  std::vector<std::vector<Eigen::Vector3d>> rob_pts;
  std::size_t num_pts = getPtsOnRobotSurface(robot_state, rob_pts);
  // std::cout << "getPtsOnRobotSurface num_pts: " << num_pts << std::endl;

  vis_data_->setTotalNumRepulsePts(num_pts);
  std::vector<Eigen::Vector3d> link_to_obs_vec = getLinkToObsVec(rob_pts);

  Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group_);

  // std::vector<ManipulabilityMeasures> manip_per_joint;

  // std::cout << "jacobian:\n " << jacobian << std::endl;
  // Eigen::VectorXd d_q_out = toEigen(joint_angles);
  Eigen::VectorXd d_q_out = Eigen::VectorXd::Zero(dof_);
  for (std::size_t i = 0; i < dof_; i++) {
    Eigen::MatrixXd link_jac = jacobian.block(0, 0, 3, i + 1);
    // std::cout << "link_jac:\n " << link_jac << std::endl;

    Eigen::MatrixXd jac_pinv_;
    utilities::pseudoInverse(link_jac, jac_pinv_);
    // std::cout << "jac_pinv_:\n " << jac_pinv_ << std::endl;
    Eigen::Vector3d vec = link_to_obs_vec[i];
    // std::cout << "i: vec: \n " << i << ": " << vec.norm() << std::endl;

    Eigen::Vector3d rob_vec =
        utilities::toEigen(std::vector<double>{vec[0], vec[1], vec[2]});

    // Eigen::MatrixXd j_t = link_jac.transpose();
    // Eigen::MatrixXd j_sq = link_jac * j_t;

    // Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(j_sq);
    // ManipulabilityMeasures manip;
    // manip.eigen_values = eigensolver.eigenvalues().real();
    // manip.eigen_vectors = eigensolver.eigenvectors().real();

    // bool is_valid = false;
    // for (std::size_t i = 0; i < manip.eigen_values.size(); i++) {
    //   double eig_val = manip.eigen_values(i);
    //   // std::cout << "eig_val " << eig_val << std::endl;
    //   Eigen::Vector3d eig_vec = manip.getVector(i);
    //   // double ab = eig_vec.dot(rob_vec);
    //   // double aa = eig_vec.dot(eig_vec);
    //   // double bb = rob_vec.dot(rob_vec);

    //   double sim1 = std::pow(eig_vec.dot(rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * rob_vec.dot(rob_vec));
    //   double sim2 = std::pow(eig_vec.dot(-1 * rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * -1 * rob_vec.dot(-1 *
    //                 rob_vec));
    //   double sim = std::max(sim1, sim2);
    //   std::cout << "eig_vec " << eig_vec.transpose() << std::endl;
    //   std::cout << "rob_vec " << rob_vec.transpose() << std::endl;
    //   std::cout << "sim " << sim << std::endl;

    //   const double VAL_THRESH = 0.2;
    //   const double SIM_THRESH = 0.2;
    //   if (eig_val > VAL_THRESH && sim > SIM_THRESH) {
    //     std::cout << "thresh passed" << std::endl;
    //     is_valid = true;
    //   }
    // }

    // is_valid = true;
    // manip.pass = is_valid;
    // manip_per_joint.emplace_back(manip);

    // double repulse_angle = 0.0;
    // if (is_valid) {
    Eigen::VectorXd d_q = jac_pinv_ * rob_vec;
    // repulse_angle = d_q[i];
    // }

    // d_q_out[i] = d_q[i];

    for (std::size_t k = 0; k < d_q.size(); k++) {
      d_q_out[k] += d_q[k];
    }

    // std::cout << "d_q:\n" << d_q.transpose() << std::endl;
    // std::cout << "d_q_out:\n " << d_q_out.transpose() << std::endl;
  }

  Eigen::VectorXd vel_out = jacobian * d_q_out;
  // std::cout << "vel_out:\n " << vel_out.transpose() << std::endl;

  // d_q_out = d_q_out * 0.5;
  // d_q_out.normalize();

  // manipulability_.emplace_back(manip_per_joint);
  vis_data_->saveRepulseAngles(joint_angles, d_q_out);
  sample_state_count_++;
  // std::cout << "d_q_out.norm():\n " << d_q_out.norm() << std::endl;
  // std::cout << "d_q_out:\n " << d_q_out.transpose() << std::endl;

  return d_q_out;
}

Eigen::VectorXd ContactPlanner::goalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = joint_goal_pos_[0] - x[0];
  v[1] = joint_goal_pos_[1] - x[1];
  v[2] = joint_goal_pos_[2] - x[2];
  v[3] = joint_goal_pos_[3] - x[3];
  v[4] = joint_goal_pos_[4] - x[4];
  v[5] = joint_goal_pos_[5] - x[5];
  v[6] = joint_goal_pos_[6] - x[6];
  v.normalize();
  return v;
}

Eigen::VectorXd ContactPlanner::negGoalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = x[0] - joint_goal_pos_[0];
  v[1] = x[1] - joint_goal_pos_[1];
  v[2] = x[2] - joint_goal_pos_[2];
  v[3] = x[3] - joint_goal_pos_[3];
  v[4] = x[4] - joint_goal_pos_[4];
  v[5] = x[5] - joint_goal_pos_[5];
  v[6] = x[6] - joint_goal_pos_[6];
  v.normalize();
  return v;
}

Eigen::VectorXd ContactPlanner::totalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd goal_vec = goalField(state);
  std::cout << "\ngoal_vec\n" << goal_vec.transpose() << std::endl;
  Eigen::VectorXd obstacle_vec = obstacleFieldConfigSpace(state);
  std::cout << "obstacle_vec\n" << obstacle_vec.transpose() << std::endl;
  Eigen::VectorXd total_vec = goal_vec + obstacle_vec;
  std::cout << "total_vec\n" << total_vec.transpose() << std::endl;
  total_vec.normalize();
  return total_vec;
}

void ContactPlanner::changePlanner() {
  planner_id_ = planner_name_;
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  ompl::geometric::PathSimplifierPtr simplifier =
      simple_setup->getPathSimplifier();

  std::function<Eigen::VectorXd(const ompl::base::State*)> vFieldFunc;

  std::function<Eigen::VectorXd(const ompl::base::State*,
                                const ompl::base::State*)>
      vFieldFuncDuo;

  if (objective_name_ == "UpstreamCost") {
    ROS_INFO_NAMED(LOGNAME, "Using UpstreamCost optimization objective.");
    vFieldFunc = std::bind(&ContactPlanner::obstacleFieldConfigSpace, this,
                           std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::VFUpstreamCriterionOptimizationObjective>(
            si, vFieldFunc);
  } else if (objective_name_ == "FieldMagnitude") {
    ROS_INFO_NAMED(LOGNAME, "Using FieldMagnitude optimization objective.");
    vFieldFunc = std::bind(&ContactPlanner::obstacleFieldTaskSpace, this,
                           std::placeholders::_1);
    optimization_objective_ =
        std::make_shared<ompl::base::VFMagnitudeOptimizationObjective>(
            si, vFieldFunc);
  } else if (objective_name_ == "FieldAlign") {
    ROS_INFO_NAMED(LOGNAME, "Using FieldAlign optimization objective.");
    vFieldFuncDuo = std::bind(&ContactPlanner::obstacleFieldCartesian, this,
                              std::placeholders::_1, std::placeholders::_2);
    optimization_objective_ =
        std::make_shared<ompl::base::VFUpstreamCriterionOptimizationObjective>(
            si, vFieldFunc);
  } else {
    ROS_ERROR_NAMED(LOGNAME, "Invalid optimization objective.");
  }

  optimization_objective_->setCostToGoHeuristic(
      &ompl::base::goalRegionCostToGo);
  simple_setup->setOptimizationObjective(optimization_objective_);

  ompl::base::PlannerPtr planner;
  if (planner_name_ == "ClassicTRRT") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: ClassicTRRT.");
    planner = std::make_shared<ompl::geometric::ClassicTRRT>(
        simple_setup->getSpaceInformation());
  } else if (planner_name_ == "ContactTRRT") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: ContactTRRT.");
    planner = std::make_shared<ompl::geometric::ContactTRRT>(
        simple_setup->getSpaceInformation(), vFieldFunc);
  } else if (planner_name_ == "ContactTRRTDuo") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: ContactTRRTDuo.");
    planner = std::make_shared<ompl::geometric::ContactTRRT>(
        simple_setup->getSpaceInformation(), vFieldFuncDuo);
  } else if (planner_name_ == "BITstar") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: BITstar.");
    planner = std::make_shared<ompl::geometric::BITstar>(
        simple_setup->getSpaceInformation());
  } else if (planner_name_ == "RRTstar") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: RRTstar.");
    planner = std::make_shared<ompl::geometric::RRTstar>(
        simple_setup->getSpaceInformation());
  } else if (planner_name_ == "FMT") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: FMT.");
    planner = std::make_shared<ompl::geometric::FMT>(
        simple_setup->getSpaceInformation());
  } else if (planner_name_ == "VFRRT") {
    ROS_INFO_NAMED(LOGNAME, "Using planner: VFRRT.");
    double exploration = 0.7;
    double initial_lambda = 1.0;
    std::size_t update_freq = 10.0;
    planner = std::make_shared<ompl::geometric::VFRRT>(
        simple_setup->getSpaceInformation(), vFieldFunc, exploration,
        initial_lambda, update_freq);
  } else {
    ROS_ERROR_NAMED(LOGNAME, "Invalid planner.");
  }

  simple_setup->setPlanner(planner);
}

std::vector<Eigen::Vector3d> ContactPlanner::getSimObstaclePos() {
  return sim_obstacle_pos_;
};

void ContactPlanner::analyzePlanResponse(BenchMarkData& benchmark_data) {
  for (auto sphere : spherical_obstacles_) {
    contact_perception_->addSphere(sphere.first, sphere.second);
  }

  PlanAnalysisData& plan_analysis = benchmark_data.plan_analysis;

  // collision_tools.h can help with visualization
  moveit_msgs::MotionPlanResponse msg;
  plan_response_.getMessage(msg);
  std::size_t num_pts = msg.trajectory.joint_trajectory.points.size();
  plan_analysis.num_path_states = num_pts;
  ROS_INFO_NAMED(LOGNAME, "Trajectory num_pts: %ld", num_pts);

  planning_scene_monitor::LockedPlanningSceneRW(psm_)->addCollisionDetector(
      collision_detection::CollisionDetectorAllocatorBullet::create());

  planning_scene_monitor::LockedPlanningSceneRW(psm_)
      ->setActiveCollisionDetector("Bullet");

  const std::string active_col_det =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)
          ->getActiveCollisionDetectorName();
  ROS_INFO_NAMED(LOGNAME, "active_col_det: %s", active_col_det.c_str());

  std::vector<std::string> col_det_names;
  planning_scene_monitor::LockedPlanningSceneRO(psm_)
      ->getCollisionDetectorNames(col_det_names);
  for (auto name : col_det_names) {
    std::cout << "active col det name: " << name << std::endl;
  }

  const collision_detection::CollisionEnvConstPtr collision_env =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)
          ->getCollisionEnvUnpadded();

  collision_detection::CollisionRequest collision_request;
  collision_request.distance = false;
  collision_request.cost = false;
  collision_request.contacts = true;
  collision_request.max_contacts = 20;
  collision_request.max_contacts_per_pair = 1;
  collision_request.max_cost_sources = 20;
  collision_request.verbose = false;

  collision_detection::DistanceRequest distance_request;
  distance_request.type = collision_detection::DistanceRequestType::ALL;
  distance_request.enable_nearest_points = true;
  distance_request.enable_signed_distance = true;
  distance_request.distance_threshold = 0.05;
  distance_request.verbose = true;
  distance_request.max_contacts_per_body = 1;
  distance_request.compute_gradient = false;
  distance_request.enableGroup(psm_->getPlanningScene()->getRobotModel());

  collision_detection::AllowedCollisionMatrix acm =
      planning_scene_monitor::LockedPlanningSceneRO(psm_)
          ->getAllowedCollisionMatrix();
  distance_request.acm = &acm;

  moveit::core::RobotState prev_robot_state(
      psm_->getPlanningScene()->getRobotModel());
  Eigen::Vector3d prev_tip_pos;

  for (std::size_t pt_idx = 0; pt_idx < num_pts; pt_idx++) {
    // ROS_INFO_NAMED(LOGNAME, "Analyzing trajectory point number: %ld",
    // pt_idx);

    trajectory_msgs::JointTrajectoryPoint point =
        msg.trajectory.joint_trajectory.points[pt_idx];
    std::vector<double> joint_angles(dof_, 0.0);
    for (std::size_t jnt_idx = 0; jnt_idx < dof_; jnt_idx++) {
      joint_angles[jnt_idx] = point.positions[jnt_idx];
    }

    moveit::core::RobotState robot_state(
        psm_->getPlanningScene()->getRobotModel());
    robot_state.setJointGroupPositions(joint_model_group_, joint_angles);
    robot_state.update();

    std::vector<std::string> tips;
    joint_model_group_->getEndEffectorTips(tips);
    Eigen::Isometry3d tip_tf =
        robot_state.getGlobalLinkTransform("panda_link8");
    Eigen::Vector3d tip_pos{tip_tf.translation().x(), tip_tf.translation().y(),
                            tip_tf.translation().z()};
    vis_data_->ee_path_pts_.emplace_back(tip_pos);

    if (pt_idx > 0) {
      double dist_travelled = robot_state.distance(prev_robot_state);
      ROS_INFO_NAMED(LOGNAME, "dist_travelled rad: %f", dist_travelled);
      plan_analysis.joint_path_len += dist_travelled;

      double tip_travelled = utilities::getDistance(tip_pos, prev_tip_pos);
      ROS_INFO_NAMED(LOGNAME, "tip_travelled m: %f", tip_travelled);
      plan_analysis.ee_path_len += tip_travelled;
    }
    prev_tip_pos = tip_pos;
    prev_robot_state = robot_state;

    // COLLISION

    collision_detection::CollisionResult collision_result;
    planning_scene_monitor::LockedPlanningSceneRO(psm_)->checkCollisionUnpadded(
        collision_request, collision_result, robot_state);
    bool collision = collision_result.collision;

    std::size_t contact_count = collision_result.contact_count;
    plan_analysis.total_contact_count += contact_count;
    if (contact_count > 0) {
      plan_analysis.num_contact_states += 1;
    }

    double distance = collision_result.distance;
    // ROS_INFO_NAMED(LOGNAME, "distance: %f", distance);

    collision_detection::CollisionResult::ContactMap contact_map =
        collision_result.contacts;

    double depth_per_state = 0;
    std::vector<double> depth_per_link(9, 0);

    for (auto contact : contact_map) {
      std::size_t num_subcontacts = contact.second.size();
      // ROS_INFO_NAMED(LOGNAME, "Number of subcontacts: %ld", num_subcontacts);
      for (std::size_t subc_idx = 0; subc_idx < num_subcontacts; subc_idx++) {
        collision_detection::Contact subcontact = contact.second[subc_idx];
        ROS_INFO_NAMED(LOGNAME, "Body 1: %s", subcontact.body_name_1.c_str());
        ROS_INFO_NAMED(LOGNAME, "Body 2: %s", subcontact.body_name_2.c_str());
        ROS_INFO_NAMED(LOGNAME, "Depth: %f", subcontact.depth);
        if (std::abs(subcontact.depth) > 1000) {
          continue;
        }

        plan_analysis.total_contact_depth += std::abs(subcontact.depth);
        depth_per_state += std::abs(subcontact.depth);

        std::size_t idx = 0;
        if (linkNameToIdx(subcontact.body_name_1, idx)) {
          depth_per_link[idx] = std::abs(subcontact.depth);
        }
      }
    }
    plan_analysis.trajectory_analysis.total_depth.emplace_back(depth_per_state);
    plan_analysis.trajectory_analysis.depth_per_link.emplace_back(
        depth_per_link);
    std::set<collision_detection::CostSource> cost_sources =
        collision_result.cost_sources;
  }

  ROS_INFO_NAMED(LOGNAME, "plan_analysis.total_contact_depth: %f",
                 plan_analysis.total_contact_depth);
  ROS_INFO_NAMED(LOGNAME, "plan_analysis.joint_path_len: %f",
                 plan_analysis.joint_path_len);
  ROS_INFO_NAMED(LOGNAME, "plan_analysis.ee_path_len: %f",
                 plan_analysis.ee_path_len);
  ROS_INFO_NAMED(LOGNAME, "plan_analysis.num_contact_states: %ld",
                 plan_analysis.num_contact_states);
  ROS_INFO_NAMED(LOGNAME, "plan_analysis.num_path_states: %ld",
                 plan_analysis.num_path_states);
  ROS_INFO_NAMED(LOGNAME, "plan_analysis.total_contact_count: %ld",
                 plan_analysis.total_contact_count);
}

bool ContactPlanner::linkNameToIdx(const std::string& link_name,
                                   std::size_t& idx) {
  std::vector<std::string> link_names{
      "panda_link0", "panda_link1", "panda_link2", "panda_link3", "panda_link4",
      "panda_link5", "panda_link6", "panda_link7", "panda_hand"};
  auto it = std::find(link_names.begin(), link_names.end(), link_name);
  if (it == link_names.end()) {
    ROS_ERROR_NAMED(LOGNAME, "Unable to find the following link in model: %s",
                    link_name.c_str());
    return false;
  }
  idx = std::distance(link_names.begin(), it);
  ROS_INFO_NAMED(LOGNAME, "link_name: %s, idx %ld", link_name.c_str(), idx);
  return true;
}

}  // namespace tacbot