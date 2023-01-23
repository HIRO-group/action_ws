#ifndef TACBOT_MANIPULABILITY_MEASURES_H
#define TACBOT_MANIPULABILITY_MEASURES_H

#include <Eigen/Core>
#include <iostream>

namespace tacbot {

struct ManipulabilityMeasures {
  Eigen::MatrixXd eigen_values;
  Eigen::MatrixXd eigen_vectors;

  Eigen::Vector3d getVector(std::size_t i) {
    Eigen::Vector3d eig_vec(eigen_vectors(0, i), eigen_vectors(1, i),
                            eigen_vectors(2, i));
    return eig_vec;
  }
  bool pass = false;
};

}  // namespace tacbot

#endif