

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include <fstream>
#include <iostream>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/solvers/get_program_type.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

// GUROBI
// #include "gurobi_c++.h"

using namespace drake;
using namespace solvers;
using drake::multibody::RigidBody;
using drake::systems::Adder;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::kVectorValued;
using drake::systems::controllers::InverseDynamics;
using Eigen::Translation3d;
using Eigen::Vector3d;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using systems::ApplySimulatorConfig;
using systems::BasicVector;
using systems::Context;
using systems::Simulator;
using systems::SimulatorConfig;
using systems::controllers::InverseDynamicsController;

void SetPidGains(Eigen::VectorXd* kp, Eigen::VectorXd* ki,
                 Eigen::VectorXd* kd) {
  // *kp << 100, 100, 100, 100, 100, 100, 100;
  // *ki << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  // *kd = *kp / 2.;

  kp->resize(7);
  *kp << 100, 100, 100, 100, 100, 100, 100;
  kd->resize(kp->size());
  for (int i = 0; i < kp->size(); i++) {
    // Critical damping gains.
    (*kd)[i] = 2 * std::sqrt((*kp)[i]);
  }
  *ki = Eigen::VectorXd::Zero(7);
}

// ======================
// GLOBAL VARIABLES
// ======================

MultibodyPlantConfig plant_config_;

multibody::MultibodyPlant<double>* controller_plant_;

multibody::MultibodyPlant<double>* plant_;
systems::DiagramBuilder<double>* builder_(
    new systems::DiagramBuilder<double>());
geometry::SceneGraph<double>* scene_graph_;
std::unique_ptr<drake::systems::Diagram<double>> diagram_;
std::vector<multibody::ModelInstanceIndex> robot_idx_;

systems::Simulator<double>* simulator_;

// ======================
// PRINT HELPERS
// ======================

void printPlantInfo(MultibodyPlant<double>* plant) {
  std::cout << "plant->num_actuators(): " << plant->num_actuators()
            << std::endl;
  std::cout << "plant->num_bodies(): " << plant->num_bodies() << std::endl;
  std::cout << "plant->num_positions(): " << plant->num_positions()
            << std::endl;
  std::cout << "plant->num_velocities(): " << plant->num_velocities()
            << std::endl;

  const VectorX<double> lower_limits = plant->GetPositionLowerLimits();
  const VectorX<double> upper_limits = plant->GetPositionUpperLimits();

  std::cout << "lower_limits\n" << lower_limits.transpose() << std::endl;
  std::cout << "upper_limits\n" << upper_limits.transpose() << std::endl;

  for (drake::multibody::BodyIndex body_index(0);
       body_index < plant->num_bodies(); ++body_index) {
    const drake::multibody::Body<double>& body = plant->get_body(body_index);
    std::cout << "body name: " << body.name() << std::endl;
  }
}

void printGeometryInfo(geometry::SceneGraph<double>* scene_graph) {
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph->model_inspector();

  std::size_t num_geom = inspector.num_geometries();
  std::cout << "num_geom: " << num_geom << std::endl;
  std::vector<geometry::GeometryId> allIds = inspector.GetAllGeometryIds();
  std::cout << "allIds.size(): " << allIds.size() << std::endl;

  for (std::size_t i = 0; i < allIds.size(); i++) {
    geometry::GeometryId geo = allIds[i];
    std::cout << "name: " << inspector.GetName(geo) << std::endl;
    // const geometry::ProximityProperties* props =
    //     inspector.GetProximityProperties(geo);  // nullptr unless specified
    //     in urdf or sdf
  }
}

void printContactInfo(
    MultibodyPlant<double>* plant,
    const multibody::PointPairContactInfo<double>& contact_info) {
  std::cout << "\nbodyA, bodyB: "
            << plant_->get_body(contact_info.bodyA_index()).name() << ", "
            << plant_->get_body(contact_info.bodyB_index()).name() << std::endl;
  std::cout << "slip speed: " << contact_info.slip_speed() << std::endl;
  std::cout << "separation speed: " << contact_info.separation_speed()
            << std::endl;
  std::cout << "contact point: " << contact_info.contact_point().transpose()
            << std::endl;
  std::cout << "contact force: " << contact_info.contact_force().transpose()
            << std::endl;
}

// ======================
// COST FUNCTION
// ======================

class ContactCost : public Cost {
 public:
  ContactCost() : Cost(7) {
    // this needs to be a number, not a variable otherwise runtime error occurs
  }

 private:
  std::size_t dim_ = 7;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    y->resize(1);

    // simulator_->Initialize(); // this adds significant time, and not sure is
    // needed simulator_->get_mutable_context().SetTime(0);

    auto x_vec = drake::math::ExtractValue(x);
    std::cout << "x_vec: " << x_vec.transpose() << std::endl;

    Eigen::VectorXd joint_position(dim_);
    for (std::size_t i = 0; i < dim_; i++) {
      joint_position[i] = x_vec(i);
    }

    systems::Context<double>& root_context = simulator_->get_mutable_context();

    plant_->SetPositions(
        &diagram_->GetMutableSubsystemContext(*plant_, &root_context),
        robot_idx_.at(0), joint_position);

    const drake::multibody::ContactResults<double>& contact_results =
        plant_->get_contact_results_output_port()
            .Eval<drake::multibody::ContactResults<double>>(
                diagram_->GetMutableSubsystemContext(*plant_, &root_context));

    std::cout << "num pair contacts: "
              << contact_results.num_point_pair_contacts() << std::endl;

    double cost = 0;

    for (std::size_t i = 0; i < contact_results.num_point_pair_contacts();
         ++i) {
      const auto& contact_info = contact_results.point_pair_contact_info(i);
      cost += contact_info.contact_force().norm();
      printContactInfo(plant_, contact_info);
    }

    std::cout << "cost: " << cost << std::endl;

    (*y)(0) = cost;
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    std::cout << "ContactCost DoEval 1" << std::endl;
    throw std::runtime_error("error");
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    std::cout << "ContactCost DoEval 2" << std::endl;
    throw std::runtime_error("error");
  }
};

// ======================
// CONSTRAINT FUNCTION
// ======================

class ContactConstraint : public Constraint {
 public:
  ContactConstraint()
      : Constraint(3, 7, Eigen::Vector3d(0.0, 0.0, 0.0),
                   Eigen::Vector3d(0.0, 0.0, 0.0)) {}

 private:
  std::size_t dim_ = 7;

  template <typename T, typename S>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<S>* y) const {
    y->resize(3);

    Eigen::VectorXd joint_position(dim_);
    for (std::size_t i = 0; i < dim_; i++) {
      joint_position[i] = x(i);
    }
    std::cout << "joint_position: " << joint_position.transpose() << std::endl;

    systems::Context<double>& root_context = simulator_->get_mutable_context();
    plant_->SetPositions(
        &diagram_->GetMutableSubsystemContext(*plant_, &root_context),
        robot_idx_.at(0), joint_position);

    const multibody::Frame<double>& frame_e =
        plant_->GetFrameByName("panda_leftfinger");

    const multibody::Frame<double>& frame_c =
        plant_->GetFrameByName("cylinder_base");

    math::RigidTransform<double> X_AB = plant_->CalcRelativeTransform(
        diagram_->GetMutableSubsystemContext(*plant_, &root_context), frame_e,
        frame_c);

    std::cout << "X_AB: " << X_AB.translation().transpose() << std::endl;

    (*y)(0) = static_cast<double>(X_AB.translation().x());
    (*y)(1) = static_cast<double>(X_AB.translation().y());
    (*y)(2) = static_cast<double>(X_AB.translation().z());
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    Eigen::Ref<const VectorX<double>> x_vec = drake::math::ExtractValue(x);
    DoEvalGeneric(x_vec, y);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    throw std::runtime_error("error");
  }
};

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

std::vector<multibody::ModelInstanceIndex> addRobot(
    multibody::MultibodyPlant<double>* plant) {
  multibody::Parser parser(plant);
  std::vector<multibody::ModelInstanceIndex> panda_instance =
      parser.AddModels("../../franka_description/urdf/panda_arm_hand.urdf");
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"));
  return panda_instance;
}

std::vector<multibody::ModelInstanceIndex> addTable(
    multibody::MultibodyPlant<double>* plant) {
  multibody::Parser parser(plant);
  std::vector<multibody::ModelInstanceIndex> table_instance = parser.AddModels(
      "../../manipulation_station/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");

  const double table_height = 0.7645;
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("table_link"),
                    drake::math::RigidTransformd(
                        Eigen::Vector3d{0, 0, -table_height - 0.01}));

  // const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(0),
  //                            Vector3d(0.48, 0.1, 0.3));
  // const auto& objects_frame_O = plant->AddFrame(
  //     std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
  //         "objects_frame", plant->GetFrameByName("table_top_center"), X_TO));
  return table_instance;
}

std::vector<multibody::ModelInstanceIndex> addObject(
    multibody::MultibodyPlant<double>* plant) {
  multibody::Parser parser(plant);
  std::vector<drake::multibody::ModelInstanceIndex> cylinder_instance =
      parser.AddModels(
          "../../manipulation_station/models/objects/simple_cylinder.urdf");
  return cylinder_instance;
}

int main() {
  // ======================
  // MODEL
  // ======================

  multibody::AddMultibodyPlantSceneGraphResult<double> mbp_result =
      multibody::AddMultibodyPlant(plant_config_, builder_);
  plant_ = std::move(&mbp_result.plant);
  scene_graph_ = std::move(&mbp_result.scene_graph);
  plant_->set_name("plant");
  // scene_graph_->set_name("scene_graph");

  robot_idx_ = addRobot(plant_);
  std::cout << "robot_idx_.size(): " << robot_idx_.size() << std::endl;

  std::vector<multibody::ModelInstanceIndex> table_idx;
  table_idx = addTable(plant_);
  std::cout << "table_idx.size(): " << table_idx.size() << std::endl;

  std::vector<multibody::ModelInstanceIndex> cyl_idx;
  cyl_idx = addObject(plant_);
  std::cout << "cyl_idx.size(): " << cyl_idx.size() << std::endl;

  controller_plant_ = new multibody::MultibodyPlant<double>(0.0);
  controller_plant_->set_name("controller_plant");
  // scene_graph_->set_name("Controller_scene_graph");
  std::vector<multibody::ModelInstanceIndex> control_idx;
  control_idx = addRobot(controller_plant_);
  std::cout << "control_idx.size(): " << control_idx.size() << std::endl;

  // ======================
  // FINALIZE PLANT
  // ======================

  std::cout << "full plant" << std::endl;
  plant_->Finalize();
  printPlantInfo(plant_);

  std::size_t num_instantes = plant_->num_model_instances();
  std::cout << "num_instantes: " << num_instantes << std::endl;

  std::cout << "controller plant" << std::endl;
  controller_plant_->Finalize();
  printPlantInfo(controller_plant_);
  num_instantes = controller_plant_->num_model_instances();
  std::cout << "num_instantes: " << num_instantes << std::endl;

  // ======================
  // CONTROL
  // ======================

  Eigen::VectorXd desired(14);
  desired << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006, 0.713265,
      -0.0827766, 0, 0, 0, 0, 0, 0, 0;
  drake::systems::ConstantVectorSource<double>* target =
      builder_->AddSystem<drake::systems::ConstantVectorSource>(desired);
  target->set_name("robot_target");

  const int dim = controller_plant_->num_positions();
  Eigen::VectorXd kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto idc_controller = builder_->AddSystem<InverseDynamicsController>(
      *controller_plant_, kp, ki, kd, false);

  builder_->Connect(plant_->get_state_output_port(robot_idx_.at(0)),
                    idc_controller->get_input_port_estimated_state());

  // std::cout << "here 2" << std::endl;
  // builder_->Connect(plant_->get_state_output_port(),
  //                   idc_controller->get_input_port_estimated_state());

  // builder_->Connect(controller_plant_->get_state_output_port(),
  //                   idc_controller->get_input_port_desired_state());

  builder_->Connect(target->get_output_port(),
                    idc_controller->get_input_port_desired_state());

  builder_->Connect(idc_controller->get_output_port(),
                    plant_->get_actuation_input_port(robot_idx_.at(0)));
  std::cout << "finished control" << std::endl;

  // ======================
  // VISUALIZATION
  // ======================

  // params for meshcat constructor available
  std::shared_ptr<geometry::Meshcat> meshcat =
      std::make_shared<geometry::Meshcat>(7001);

  visualization::ApplyVisualizationConfig(
      visualization::VisualizationConfig{
          .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
          .enable_alpha_sliders = true,
      },
      builder_, nullptr, nullptr, nullptr, meshcat);

  drake::geometry::MeshcatVisualizerParams params;
  params.delete_on_initialization_event = false;
  auto& visualizer = drake::geometry::MeshcatVisualizerd::AddToBuilder(
      builder_, *scene_graph_, meshcat, std::move(params));

  multibody::meshcat::ContactVisualizerParams cparams;
  // cparams.newtons_per_meter = 60.0;
  multibody::meshcat::ContactVisualizerd::AddToBuilder(
      builder_, *plant_, meshcat, std::move(cparams));
  // ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph_;

  diagram_ = builder_->Build();
  std::cout << "finished visualization" << std::endl;

  // ======================
  // CONTEXT
  // ======================

  std::unique_ptr<systems::Context<double>> plant_context =
      controller_plant_->CreateDefaultContext();

  std::cout << "has_only_continuous_state: "
            << plant_context->has_only_continuous_state() << std::endl;
  std::cout << "has_only_discrete_state: "
            << plant_context->has_only_discrete_state() << std::endl;

  // std::unique_ptr<Context<double>> context_ =
  // diagram_->CreateDefaultContext();

  // ======================
  // SIMULATION
  // ======================

  // SimulatorConfig sim_config;
  simulator_ = new systems::Simulator<double>(*diagram_);
  // ApplySimulatorConfig(sim_config, &simulator);
  systems::Context<double>& root_context = simulator_->get_mutable_context();

  Eigen::VectorXd initial_position(controller_plant_->num_positions());
  initial_position << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006,
      0.713265, -0.0827766;
  controller_plant_->SetPositions(plant_context.get(), initial_position);

  plant_->SetPositions(
      &diagram_->GetMutableSubsystemContext(*plant_, &root_context),
      robot_idx_.at(0), initial_position);

  const math::RigidTransform<double> X_WF1 = math::RigidTransform<double>(
      math::RollPitchYaw(0.0, 0.0, 0.0), Eigen::Vector3d(0.2, 0.0, 0.5));
  const auto& base_link = plant_->GetBodyByName("cylinder_base");
  plant_->SetFreeBodyPose(
      &diagram_->GetMutableSubsystemContext(*plant_, &root_context), base_link,
      X_WF1);

  simulator_->set_target_realtime_rate(1.0);
  simulator_->Initialize();
  // meshcat->StartRecording();
  // simulator_->AdvanceTo(0.01);
  // meshcat->StopRecording();
  // meshcat->PublishRecording();

  // systems::PrintSimulatorStatistics(*simulator_);

  // const std::string html_filename("meshcat_static.html");
  // std::ofstream html_file(html_filename);
  // html_file << meshcat->StaticHtml();
  // html_file.close();

  // const drake::multibody::ContactResults<double>& contact_results =
  //     plant_->get_contact_results_output_port()
  //         .Eval<drake::multibody::ContactResults<double>>(
  //             diagram_->GetMutableSubsystemContext(*plant_, &root_context));

  // std::cout << "num pair contacts: "
  //           << contact_results.num_point_pair_contacts() << std::endl;
  // std::cout << "num hydro contacts: "
  //           << contact_results.num_hydroelastic_contacts() << std::endl;

  // for (std::size_t i = 0; i < contact_results.num_point_pair_contacts(); ++i)
  // {
  //   const auto& contact_info = contact_results.point_pair_contact_info(i);
  //   printContactInfo(plant_, contact_info);
  // }

  // ======================
  // OPTIMIZATION
  // ======================
  SnoptSolver test_solver;
  drake::solvers::MathematicalProgram test_program;
  auto x_var = test_program.NewContinuousVariables<2>();

  test_program.AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), x_var);
  // test_program.AddCost(std::make_shared<DummyCost>(),
  //                      Vector3<symbolic::Variable>(x_(0), x_(1), x_(1)));
  // test_program.AddConstraint(std::make_shared<DummyConstraint>(), x_var);

  test_program.AddCost(
      std::make_shared<DummyCost>(),
      Vector3<symbolic::Variable>(x_var(0), x_var(1), x_var(1)));
  test_program.AddConstraint(
      std::make_shared<DummyConstraint>(),
      Vector3<symbolic::Variable>(x_var(0), x_var(0), x_var(1)));

  auto test_result = test_solver.Solve(test_program, {}, {});

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

  // NloptSolver solver;
  SnoptSolver solver;
  // solvers::GurobiSolver solver;

  drake::solvers::MathematicalProgram prog;
  auto q_var = prog.NewContinuousVariables<7>("q");

  Eigen::Matrix<double, 7, 1> q_guess;
  q_guess << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006, 0.713265,
      -0.0827766;
  prog.SetInitialGuess(q_var, q_guess);

  Eigen::Matrix<double, 7, 1> q_lower, q_upper;
  q_lower << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_upper << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  prog.AddBoundingBoxConstraint(q_lower, q_upper, q_var);

  // prog.AddCost(std::make_shared<ContactCost>(), q_var);
  prog.AddConstraint(std::make_shared<ContactConstraint>(), q_var);

  const ProgramType prog_type = GetProgramType(prog);
  std::cout << "prog_type: " << prog_type << std::endl;
  SolverOptions options;
  // options.SetOption(CommonSolverOption::kPrintToConsole, 1);
  options.SetOption(CommonSolverOption::kPrintFileName, "solver.txt");
  // auto result = Solve(prog, {}, {});

  MathematicalProgramResult result = solver.Solve(prog, {}, options);

  std::vector<std::string> names = result.GetInfeasibleConstraintNames(prog);

  const auto q_res = result.GetSolution(q_var);

  // Check if the solution is successful
  if (result.is_success()) {
    std::cout << "Solution found!" << std::endl;
    for (int i = 0; i < 7; i++) {
      std::cout << "q" << i + 1 << " = " << result.GetSolution(q_var[i])
                << std::endl;
    }
  } else {
    std::cout << "Failed to find a solution." << std::endl;
  }

  return 0;
}

// ======================
// COLLISION OBJECT
// ======================

// drake::multibody::ModelInstanceIndex sphere_instance =
//     plant_->AddModelInstance("Sphere1InstanceName");

// const RigidBody<double>& sphere1 = plant_->AddRigidBody(
//     "Sphere1", sphere_instance,
//     drake::multibody::SpatialInertia<double>::MakeUnitary());
// drake::multibody::CoulombFriction<double> sphere1_friction(0.8, 0.5);
// // estimated parameters for mass=1kg, penetration_tolerance=0.01m
// // and gravity g=9.8 m/s^2.
// const double sphere1_stiffness = 980;    // N/m
// const double sphere1_dissipation = 3.2;  // s/m
// double radius = 0.5;
// geometry::ProximityProperties sphere1_properties;
// sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
//                                geometry::internal::kFriction,
//                                sphere1_friction);
// sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
//                                geometry::internal::kPointStiffness,
//                                sphere1_stiffness);
// sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
//                                geometry::internal::kHcDissipation,
//                                sphere1_dissipation);
// drake::geometry::GeometryId sphere1_id = plant_->RegisterCollisionGeometry(
//     sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
//     "collision", std::move(sphere1_properties));