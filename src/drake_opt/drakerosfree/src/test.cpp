

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
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/solvers/mathematical_program_result.h"
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

/**
 * TODO
 * instead of the mug, add a real object
 * let the object fall if the robot is not constraining it
 * add centroidal moment constraint
 * what could be the cost?
 * how do we define the object, as point cloud?
 * do we need to add trajectory optimization based on contact?
 * how do we pipe this to the robot? need to spawn a ros node.
 * how does the analogous method with traj optimization deal with whole body
 * grasping is our method superior? how can we show this?
 */

class EmptyGradientCost : public Cost {
 public:
  EmptyGradientCost() : Cost(7) {}

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    std::cout << "here 1" << std::endl;
    y->resize(1);
    (*y)(0) = x(0) + x(1) + x(2) + x(3) + x(4) + x(5) + x(6);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    std::cout << "here 2" << std::endl;
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    std::cout << "here 3" << std::endl;
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    std::cout << "here 4" << std::endl;
    DoEvalGeneric<symbolic::Expression>(x.cast<symbolic::Expression>(), y);
  }
};

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

std::unique_ptr<drake::systems::Diagram<double>> diagram_{};
systems::DiagramBuilder<double>* builder_(
    new systems::DiagramBuilder<double>());
geometry::SceneGraph<double>* scene_graph_{};
MultibodyPlant<double>* plant_{};
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
  ContactCost()
      : Cost(7) {
  }  // this needs to be a number, not a variable otherwise runtime error occurs

 private:
  std::size_t dim_ = 7;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    y->resize(1);

    // simulator_->Initialize();
    // simulator_->get_mutable_context().SetTime(0);

    auto x_vec = drake::math::ExtractValue(x);
    std::cout << "x_vec: " << x_vec.transpose() << std::endl;

    Eigen::VectorXd joint_positions(dim_);
    for (std::size_t i = 0; i < dim_; i++) {
      joint_positions[i] = x_vec(i);
    }

    systems::Context<double>& root_context = simulator_->get_mutable_context();
    plant_->SetPositions(
        &diagram_->GetMutableSubsystemContext(*plant_, &root_context),
        joint_positions);

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
      // printContactInfo(plant_, contact_info);
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

int main() {
  // ======================
  // MODEL
  // ======================

  MultibodyPlantConfig plant_config;

  multibody::AddMultibodyPlantSceneGraphResult<double> mbp_result =
      multibody::AddMultibodyPlant(plant_config, builder_);

  plant_ = std::move(&mbp_result.plant);
  scene_graph_ = std::move(&mbp_result.scene_graph);

  multibody::Parser parser(plant_);
  std::vector<drake::multibody::ModelInstanceIndex> panda_instance =
      parser.AddModels("../../franka_description/urdf/panda_arm_hand.urdf");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("panda_link0"));
  std::vector<drake::multibody::ModelInstanceIndex> table_instance =
      parser.AddModels(
          "../../manipulation_station/models/table/"
          "extra_heavy_duty_table_surface_only_collision.sdf");

  const double table_height = 0.7645;
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("table_link"),
                     drake::math::RigidTransformd(
                         Eigen::Vector3d{0, 0, -table_height - 0.01}));

  const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(0),
                             Vector3d(0.48, 0.1, 0.3));
  const auto& objects_frame_O = plant_->AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
          "objects_frame", plant_->GetFrameByName("table_top_center"), X_TO));

  std::vector<drake::multibody::ModelInstanceIndex> cylinder_instance =
      parser.AddModels("../../manipulation_station/models/cylinder.sdf");
  plant_->WeldFrames(objects_frame_O, plant_->GetFrameByName("cylinder_link"),
                     drake::math::RigidTransformd(Eigen::Vector3d{0, 0, 0.0}));

  // ======================
  // FINALIZE PLANT
  // ======================

  plant_->Finalize();
  printPlantInfo(plant_);

  // ======================
  // CONTROL
  // ======================

  Eigen::VectorXd desired(14);
  desired << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006, 0.713265,
      -0.0827766, 0, 0, 0, 0, 0, 0, 0;
  drake::systems::ConstantVectorSource<double>* target =
      builder_->template AddSystem<drake::systems::ConstantVectorSource>(
          desired);
  target->set_name("target");

  const int dim = plant_->num_positions();
  Eigen::VectorXd kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto idc_controller = builder_->AddSystem<InverseDynamicsController>(
      *plant_, kp, ki, kd, false);

  builder_->Connect(plant_->get_state_output_port(),
                    idc_controller->get_input_port_estimated_state());

  // builder.Connect(plant_->get_state_output_port(),
  //                 idc_controller->get_input_port_desired_state()); // let the
  //                 robot keep steady position, assume desired pose is equal to
  //                 initial pose

  builder_->Connect(target->get_output_port(),
                    idc_controller->get_input_port_desired_state());

  builder_->Connect(idc_controller->get_output_port(),
                    plant_->get_actuation_input_port());

  // ======================
  // VISUALIZATION
  // ======================

  std::shared_ptr<geometry::Meshcat> meshcat =
      std::make_shared<geometry::Meshcat>();  // params available

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

  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  diagram_->SetDefaultContext(diagram_context.get());
  systems::Context<double>* plant_context =
      &diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());
  std::cout << "has_only_continuous_state: "
            << plant_context->has_only_continuous_state() << std::endl;
  std::cout << "has_only_discrete_state: "
            << plant_context->has_only_discrete_state() << std::endl;
  std::cout << "get_discrete_state_vector().size(): "
            << plant_context->get_discrete_state_vector().size() << std::endl;

  // ======================
  // SIMULATION
  // ======================

  // SimulatorConfig sim_config;
  simulator_ = new systems::Simulator<double>(*diagram_);
  // ApplySimulatorConfig(sim_config, &simulator);

  systems::Context<double>& root_context = simulator_->get_mutable_context();

  Eigen::VectorXd initial_position(plant_->num_positions());
  initial_position << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006,
      0.713265, -0.0827766;

  plant_->SetPositions(
      &diagram_->GetMutableSubsystemContext(*plant_, &root_context),
      initial_position);

  simulator_->set_target_realtime_rate(1.0);
  simulator_->Initialize();
  meshcat->StartRecording();
  simulator_->AdvanceTo(1);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  systems::PrintSimulatorStatistics(*simulator_);

  const std::string html_filename("meshcat_static.html");
  std::ofstream html_file(html_filename);
  html_file << meshcat->StaticHtml();
  html_file.close();

  const drake::multibody::ContactResults<double>& contact_results =
      plant_->get_contact_results_output_port()
          .Eval<drake::multibody::ContactResults<double>>(
              diagram_->GetMutableSubsystemContext(*plant_, &root_context));

  std::cout << "num pair contacts: "
            << contact_results.num_point_pair_contacts() << std::endl;
  std::cout << "num hydro contacts: "
            << contact_results.num_hydroelastic_contacts() << std::endl;

  // ======================
  // OPTIMIZATION
  // ======================

  drake::solvers::MathematicalProgram prog;
  auto q_var = prog.NewContinuousVariables<7>();

  Eigen::Matrix<double, 7, 1> q_lower, q_upper;
  q_lower << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_upper << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  prog.AddBoundingBoxConstraint(q_lower, q_upper, q_var);

  prog.AddCost(std::make_shared<ContactCost>(), q_var);
  auto result = Solve(prog, {}, {});

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