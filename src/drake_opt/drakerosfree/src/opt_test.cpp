#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <tuple>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_type.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::RowVector2d;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::symbolic::Expression;
using std::numeric_limits;

using namespace drake;
using namespace solvers;

const double kInf = std::numeric_limits<double>::infinity();

class DummyConstraint : public Constraint {
  // 0.5x² + 0.5*y² + z² = 1
 public:
  DummyConstraint() : Constraint(1, 3, Vector1d(1), Vector1d(1)) {}

 protected:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    y->resize(1);
    (*y)(0) = 0.5 * x(0) * x(0) + 0.5 * x(1) * x(1) + x(2) * x(2);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric<double>(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric<AutoDiffXd>(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
  }
};

class DummyCost : public Cost {
  // -x²-2xy - 2xz - y² - 3z²
 public:
  DummyCost() : Cost(3) {}

 protected:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    y->resize(1);
    (*y)(0) = -x(0) * x(0) - 2 * x(0) * x(1) - 2 * x(0) * x(2) - x(1) * x(1) -
              3 * x(2) * x(2);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric<double>(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric<AutoDiffXd>(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
  }
};

int main() {
  // SnoptSolver solver;
  IpoptSolver solver;
  // NloptSolver solver;

  drake::solvers::MathematicalProgram prog;
  auto x_var = prog.NewContinuousVariables<3>();

  prog.AddBoundingBoxConstraint(0, kInf, x_var);

  prog.AddCost(std::make_shared<DummyCost>(), x_var);
  prog.AddConstraint(std::make_shared<DummyConstraint>(), x_var);

  auto test_result = solver.Solve(prog, {}, {});

  if (test_result.is_success()) {
    std::cout << "Solution found!" << std::endl;
    for (int i = 0; i < 3; i++) {
      std::cout << "x" << i + 1 << " = " << test_result.GetSolution(x_var[i])
                << std::endl;
    }
  } else {
    std::cout << "Failed to find a solution." << std::endl;
  }

  return 0;
}