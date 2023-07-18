#!/usr/bin/env python

##
#
# Contact-implicit trajectory optimization for whole-arm manipulation
# using a Kinova Gen3 manipulator arm.
#
##

import time
import numpy as np
from pydrake.all import *
from opt_eq import OptEq

# Choose what to do
simulate = True   # Run a simple simulation with fixed input
optimize = True    # Find an optimal trajectory using ilqr


####################################
# Parameters
####################################

T = 1.0
dt = 0.002
playback_rate = 0.125

# Contact model parameters
dissipation = 5.0              # controls "bounciness" of collisions: lower is bouncier
# controls "squishiness" of collisions: lower is squishier
hydroelastic_modulus = 5e6
resolution_hint = 0.05         # smaller means a finer mesh

mu_static = 0.3
mu_dynamic = 0.2

# Hydroelastic, Point, or HydroelasticWithFallback
contact_model = ContactModel.kHydroelastic
mesh_type = HydroelasticContactRepresentation.kTriangle  # Triangle or Polygon

# Some useful joint angle definitions
q_home = np.array([-2.02408, -1.06383, 1.8716, -1.80128, 0.00569006, 0.713265,
                   -0.0827766])
q_home0 = np.hstack([q_home, np.zeros(7)])
x0 = np.hstack([q_home, np.zeros(7)])


def connect_controller(controller_plant, plant, builder, target, arm_idx):

    kp = np.array([100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0])
    kd = 2.0*np.sqrt(kp)
    ki = np.zeros(7)

    controller = InverseDynamicsController(robot=controller_plant,
                                           kp=kp,
                                           ki=ki,
                                           kd=kd,
                                           has_reference_acceleration=False)
    builder.AddSystem(controller)
    builder.Connect(plant.get_state_output_port(arm_idx),
                    controller.get_input_port_estimated_state())
    builder.Connect(target.get_output_port(),
                    controller.get_input_port_desired_state())
    builder.Connect(controller.get_output_port(),
                    plant.get_actuation_input_port(arm_idx))
    return controller


def add_robot(plant):
    # (rigid hydroelastic contact included)
    urdf = "../franka_description/urdf/panda.urdf"
    arm_idx = Parser(plant).AddModelFromFile(urdf)
    X_robot = RigidTransform()
    # base attachment sets the robot up a bit
    X_robot.set_translation([0, 0, 0.015])
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("panda_link0", arm_idx),
                     X_robot)
    return plant, arm_idx


def add_table(plant):
    urdf = "../manipulation_station/models/table/extra_heavy_duty_table_surface_only_collision.sdf"
    Parser(plant).AddModelFromFile(urdf)
    table_height = 0.7645

    X_table = RigidTransform()
    # base attachment sets the robot up a bit
    X_table.set_translation([0, 0, -table_height-0.01])
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("table_link"),
                     X_table)

    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    return plant


def add_ball(plant):
    # Add a ball with compliant hydroelastic contact
    mass = 0.258
    radius = 0.1
    I = SpatialInertia(mass, np.zeros(3), UnitInertia.HollowSphere(radius))
    ball_instance = plant.AddModelInstance("ball")
    ball = plant.AddRigidBody("ball", ball_instance, I)
    X_ball = RigidTransform()
    friction = CoulombFriction(0.7*mu_static, 0.7*mu_dynamic)
    ball_props = ProximityProperties()
    AddCompliantHydroelasticProperties(
        resolution_hint, hydroelastic_modulus, ball_props)
    AddContactMaterial(dissipation=dissipation,
                       friction=friction, properties=ball_props)
    plant.RegisterCollisionGeometry(ball, X_ball, Sphere(radius),
                                    "ball_collision", ball_props)

    color = np.array([0.8, 1.0, 0.0, 0.5])
    plant.RegisterVisualGeometry(
        ball, X_ball, Sphere(radius), "ball_visual", color)

    # Choose contact model
    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    return plant


####################################
# Solve Optimization
####################################
if optimize:
    # Create a system model (w/o visualizer) to do the optimization over
    builder_ = DiagramBuilder()
    plant_, scene_graph_ = AddMultibodyPlantSceneGraph(builder_, dt)

    plant_, _ = add_robot(plant_)
    plant_ = add_table(plant_)
    plant_.Finalize()

    builder_.ExportInput(plant_.get_actuation_input_port(), "control")
    system_ = builder_.Build()

    optEq = OptEq(system_)
    optEq.SetInitialState(q_home)
    q = optEq.Solve()
    x0 = np.hstack([q, np.zeros(7)])


####################################
# Run Simulation
####################################

if simulate:

    ####################################
    # Create system diagram
    ####################################
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)
    plant.set_name("plant")
    scene_graph.set_name("scene_graph")
    plant, arm_idx = add_robot(plant)
    plant = add_table(plant)
    plant = add_ball(plant)
    plant.Finalize()

    controller_plant = MultibodyPlant(0.0)
    controller_plant.set_name("controller_plant")
    controller_plant, _ = add_robot(controller_plant)
    controller_plant.Finalize()
    # builder.AddSystem(controller_plant)

    target = ConstantVectorSource(x0)
    target_context = target.CreateDefaultContext()
    builder.AddSystem(target)
    controller = connect_controller(
        controller_plant, plant, builder, target, arm_idx)

    # Connect to visualizer
    params = DrakeVisualizerParams(
        role=Role.kProximity, show_hydroelastic=True)
    DrakeVisualizer(params=params).AddToBuilder(builder, scene_graph)
    ConnectContactResultsToDrakeVisualizer(builder, plant, scene_graph)

    # Finalize the diagram
    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # Set initial state
    controller_context = controller_plant.CreateDefaultContext()
    controller_plant.SetPositionsAndVelocities(controller_context, q_home0)

    ball_link = plant.GetBodyByName("ball")
    context = plant.CreateDefaultContext()
    plant.SetFreeBodyPose(plant_context, ball_link,
                          RigidTransform([0, 0.15, 0.2]))

    # Simulate the system
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(playback_rate)
    simulator.set_publish_every_time_step(False)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()

    simulator.AdvanceTo(T)
