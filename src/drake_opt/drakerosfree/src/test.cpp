

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

class EmptyGradientCost : public Cost {
 public:
  EmptyGradientCost() : Cost(7) {}

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const {
    y->resize(1);
    (*y)(0) = x(0) + x(1) + x(2) + x(3) + x(4) + x(5) + x(6);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DoEvalGeneric(x, y);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
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

int main() {
  // ======================
  // MODEL
  // ======================

  systems::DiagramBuilder<double> builder;
  MultibodyPlantConfig plant_config;

  auto [plant, scene_graph] =
      multibody::AddMultibodyPlant(plant_config, &builder);
  multibody::Parser parser(&plant);
  std::vector<drake::multibody::ModelInstanceIndex> panda_instance =
      parser.AddModels("../../franka_description/urdf/panda_arm_hand.urdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));
  std::vector<drake::multibody::ModelInstanceIndex> table_instance =
      parser.AddModels(
          "../../manipulation_station/models/table/"
          "extra_heavy_duty_table_surface_only_collision.sdf");

  const double table_height = 0.7645;
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("table_link"),
                   drake::math::RigidTransformd(
                       Eigen::Vector3d{0, 0, -table_height - 0.01}));

  const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(0),
                             Vector3d(0.48, 0.1, 0.3));
  const auto& objects_frame_O = plant.AddFrame(
      std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
          "objects_frame", plant.GetFrameByName("table_top_center"), X_TO));

  std::vector<drake::multibody::ModelInstanceIndex> cylinder_instance =
      parser.AddModels("../../manipulation_station/models/cylinder.sdf");
  plant.WeldFrames(objects_frame_O, plant.GetFrameByName("cylinder_link"),
                   drake::math::RigidTransformd(Eigen::Vector3d{0, 0, 0.0}));

  // ======================
  // FINALIZE PLANT
  // ======================

  plant.Finalize();

  std::cout << "plant.num_actuators(): " << plant.num_actuators() << std::endl;
  std::cout << "plant.num_bodies(): " << plant.num_bodies() << std::endl;
  std::cout << "plant.num_positions(): " << plant.num_positions() << std::endl;
  std::cout << "plant.num_velocities(): " << plant.num_velocities()
            << std::endl;

  const VectorX<double> lower_limits = plant.GetPositionLowerLimits();
  const VectorX<double> upper_limits = plant.GetPositionUpperLimits();

  std::cout << "lower_limits\n" << lower_limits.transpose() << std::endl;
  std::cout << "upper_limits\n" << upper_limits.transpose() << std::endl;

  for (drake::multibody::BodyIndex body_index(0);
       body_index < plant.num_bodies(); ++body_index) {
    const drake::multibody::Body<double>& body = plant.get_body(body_index);
    std::cout << "body name: " << body.name() << std::endl;
  }
  const int dim = plant.num_positions();

  // ======================
  // CONTROL
  // ======================

  Eigen::VectorXd desired(14);
  desired << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006, 0.713265,
      -0.0827766, 0, 0, 0, 0, 0, 0, 0;
  drake::systems::ConstantVectorSource<double>* target =
      builder.template AddSystem<drake::systems::ConstantVectorSource>(desired);
  target->set_name("target");

  Eigen::VectorXd kp(dim), ki(dim), kd(dim);
  SetPidGains(&kp, &ki, &kd);

  auto idc_controller =
      builder.AddSystem<InverseDynamicsController>(plant, kp, ki, kd, false);

  builder.Connect(plant.get_state_output_port(),
                  idc_controller->get_input_port_estimated_state());

  // builder.Connect(plant.get_state_output_port(),
  //                 idc_controller->get_input_port_desired_state()); // let the
  //                 robot keep steady position, assume desired pose is equal to
  //                 initial pose

  builder.Connect(target->get_output_port(),
                  idc_controller->get_input_port_desired_state());

  builder.Connect(idc_controller->get_output_port(),
                  plant.get_actuation_input_port());

  // ======================
  // VISUALIZATION
  // ======================

  auto meshcat = std::make_shared<geometry::Meshcat>();
  visualization::ApplyVisualizationConfig(
      visualization::VisualizationConfig{
          .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
          .enable_alpha_sliders = true,
      },
      &builder, nullptr, nullptr, nullptr, meshcat);

  drake::geometry::MeshcatVisualizerParams params;
  params.delete_on_initialization_event = false;
  auto& visualizer = drake::geometry::MeshcatVisualizerd::AddToBuilder(
      &builder, scene_graph, meshcat, std::move(params));

  multibody::meshcat::ContactVisualizerParams cparams;
  // cparams.newtons_per_meter = 60.0;
  multibody::meshcat::ContactVisualizerd::AddToBuilder(&builder, plant, meshcat,
                                                       std::move(cparams));
  // ConnectContactResultsToDrakeVisualizer(&builder, plant, scene_graph);

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>* plant_context =
      &diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  std::cout << "has_only_continuous_state: "
            << plant_context->has_only_continuous_state() << std::endl;
  std::cout << "has_only_discrete_state: "
            << plant_context->has_only_discrete_state() << std::endl;
  std::cout << "get_discrete_state_vector().size(): "
            << plant_context->get_discrete_state_vector().size() << std::endl;

  // ======================
  // SIMULATION
  // ======================

  SimulatorConfig sim_config;
  systems::Simulator<double> simulator(*diagram);
  ApplySimulatorConfig(sim_config, &simulator);

  systems::Context<double>& root_context = simulator.get_mutable_context();

  Eigen::VectorXd initial_position(dim);
  initial_position << -2.02408, -1.06383, 1.8716, -1.80128, 0.00569006,
      0.713265, -0.0827766;

  plant.SetPositions(&diagram->GetMutableSubsystemContext(plant, &root_context),
                     initial_position);

  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  meshcat->StartRecording();
  simulator.AdvanceTo(1);
  meshcat->StopRecording();
  meshcat->PublishRecording();

  systems::PrintSimulatorStatistics(simulator);

  const std::string html_filename("meshcat_static.html");
  std::ofstream html_file(html_filename);
  html_file << meshcat->StaticHtml();
  html_file.close();

  const drake::multibody::ContactResults<double>& contact_results =
      plant.get_contact_results_output_port()
          .Eval<drake::multibody::ContactResults<double>>(
              diagram->GetMutableSubsystemContext(
                  plant, &root_context));  //*plant_context

  std::cout << "num pair contacts: "
            << contact_results.num_point_pair_contacts() << std::endl;
  std::cout << "num hydro contacts: "
            << contact_results.num_hydroelastic_contacts() << std::endl;

  std::vector<drake::multibody::SpatialForce<double>> F_BBo_W_array(
      plant.num_bodies(), drake::multibody::SpatialForce<double>{
                              Vector3d::Zero(), Vector3d::Zero()});

  for (std::size_t i = 0; i < contact_results.num_point_pair_contacts(); ++i) {
    const auto& contact_info = contact_results.point_pair_contact_info(i);

    // std::cout << "\nbodyA: "
    //           << plant.get_body(contact_info.bodyA_index()).name() <<
    //           std::endl;
    // std::cout << "bodyB: " <<
    // plant.get_body(contact_info.bodyB_index()).name()
    //           << std::endl;

    // std::cout << "slip speed: " << contact_info.slip_speed() << std::endl;
    // std::cout << "separation speed: " << contact_info.separation_speed()
    //           << std::endl;
    // std::cout << "contact point: " <<
    // contact_info.contact_point().transpose()
    //           << std::endl;
    // std::cout << "contact force: " <<
    // contact_info.contact_force().transpose()
    //           << std::endl;

    const drake::multibody::SpatialForce<double> F_Bc_W{
        Vector3d::Zero(), contact_info.contact_force()};
    const Vector3d& p_WC = contact_info.contact_point();
    const auto& bodyA = plant.get_body(contact_info.bodyA_index());
    const Vector3d& p_WAo = bodyA.EvalPoseInWorld(*plant_context).translation();
    const Vector3d& p_CAo_W = p_WAo - p_WC;
    const auto& bodyB = plant.get_body(contact_info.bodyB_index());
    const Vector3d& p_WBo = bodyB.EvalPoseInWorld(*plant_context).translation();
    const Vector3d& p_CBo_W = p_WBo - p_WC;

    // N.B. Since we are using this method to test the internal (private)
    // MultibodyPlant::EvalSpatialContactForcesContinuous(), we must use
    // internal API to generate a forces vector sorted in the same way, by
    // internal::BodyNodeIndex.
    F_BBo_W_array[bodyB.node_index()] += F_Bc_W.Shift(p_CBo_W);
    F_BBo_W_array[bodyA.node_index()] -= F_Bc_W.Shift(p_CAo_W);
  }

  // For a SceneGraph<T> instance called scene_graph.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();

  std::size_t num_geom = inspector.num_geometries();
  std::cout << "num_geom: " << num_geom << std::endl;
  std::vector<geometry::GeometryId> allIds = inspector.GetAllGeometryIds();
  std::cout << "allIds.size(): " << allIds.size() << std::endl;

  for (std::size_t i = 0; i < allIds.size(); i++) {
    geometry::GeometryId geo = allIds[i];
    std::cout << "name: " << inspector.GetName(geo) << std::endl;
    const geometry::ProximityProperties* props =
        inspector.GetProximityProperties(geo);  // nullptr
  }

  // ======================
  // OPTIMIZATION
  // ======================

  drake::solvers::MathematicalProgram prog;

  auto q_var = prog.NewContinuousVariables<7>();

  Eigen::Matrix<double, 7, 1> q_lower, q_upper;
  q_lower << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  q_upper << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  prog.AddBoundingBoxConstraint(q_lower, q_upper, q_var);

  prog.AddCost(std::make_shared<EmptyGradientCost>(), q_var);
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
//     plant.AddModelInstance("Sphere1InstanceName");

// const RigidBody<double>& sphere1 = plant.AddRigidBody(
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
// drake::geometry::GeometryId sphere1_id = plant.RegisterCollisionGeometry(
//     sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
//     "collision", std::move(sphere1_properties));