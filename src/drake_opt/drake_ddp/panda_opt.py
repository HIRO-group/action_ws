#!/usr/bin/env python

import time
import numpy as np
from pydrake.all import *
from opt_eq import OptEq
import typing


# Choose what to do
simulate = True   # Run a simple simulation with fixed input
optimize = True    # Find an optimal trajectory using ilqr


####################################
# Parameters
####################################

T = 1.0
dt = 0.01
playback_rate = 0.125

# Contact model parameters
dissipation = 5.0              # controls "bounciness" of collisions: lower is bouncier
# controls "squishiness" of collisions: lower is squishier
hydroelastic_modulus = 5e6
resolution_hint = 0.05         # smaller means a finer mesh
mu_static = 0.3
mu_dynamic = 0.2

# Hydroelastic, Point, or HydroelasticWithFallback
contact_model = ContactModel.kHydroelasticWithFallback
mesh_type = HydroelasticContactRepresentation.kTriangle  # Triangle or Polygon

# Some useful joint angle definitions
q_home = np.array([0., 0.2, 0., -2.756, 0., 1.570, 0.785])
# q_home = np.array([0., -0.785, 0., -2.356, 0., 1.570, 0.785])

q_home0 = np.hstack([q_home, np.zeros(7)])
x0 = np.hstack([q_home, np.zeros(7)])

obj_pos = np.array([0.3, 0., 0.05])
obj_ori = np.array([0., 0., 0., 1])
q_full0 = np.hstack([q_home, obj_ori, obj_pos, np.zeros(9), np.zeros(4)])
print("q_full0.size: ", q_full0.size)


def get_proximity_properties():
    props = ProximityProperties()
    friction = CoulombFriction(0.7*mu_static, 0.7*mu_dynamic)
    AddCompliantHydroelasticProperties(
        resolution_hint, hydroelastic_modulus, props)
    AddContactMaterial(dissipation=dissipation,
                       friction=friction, properties=props)
    return props


def connect_pid_controller(controller_plant, plant, builder, target, arm_idx):
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
    return arm_idx


def add_table(plant):
    urdf = "../manipulation_station/models/table/extra_heavy_duty_table_surface_only_collision.sdf"
    table_idx = Parser(plant).AddModelFromFile(urdf)
    table_height = 0.7645
    X_table = RigidTransform()
    # base attachment sets the robot up a bit
    X_table.set_translation([0, 0, -table_height-0.01])
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("table_link"),
                     X_table)

    return table_idx


def add_ball(plant):
    # Add a ball with compliant hydroelastic contact
    mass = 0.258
    radius = 0.1
    I = SpatialInertia(mass, np.zeros(3), UnitInertia.HollowSphere(radius))
    ball_idx = plant.AddModelInstance("ball")
    ball = plant.AddRigidBody("ball", ball_idx, I)
    X_ball = RigidTransform()
    friction = CoulombFriction(0.7*mu_static, 0.7*mu_dynamic)
    props = get_proximity_properties()
    plant.RegisterCollisionGeometry(ball, X_ball, Sphere(radius),
                                    "ball_collision", props)

    color = np.array([0.8, 1.0, 0.0, 0.5])
    plant.RegisterVisualGeometry(
        ball, X_ball, Sphere(radius), "ball_visual", color)
    return ball_idx


def add_ground(plant):
    # Add the ground as a big box.
    ground_idx = plant.AddModelInstance("ground")
    ground_box = plant.AddRigidBody(
        "ground", ground_idx, SpatialInertia(1, np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
    X_WG = RigidTransform([0, 0, -0.1])
    props = get_proximity_properties()
    plant.RegisterCollisionGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        props)
    plant.RegisterVisualGeometry(
        ground_box, RigidTransform(), Box(10, 10, 0.1), "ground",
        [0.5, 0.5, 0.5, 1.])
    plant.WeldFrames(plant.world_frame(), ground_box.body_frame(), X_WG)


def add_box(plant: MultibodyPlant):
    # Add boxes
    masses = [1.]
    box_sizes = [np.array([0.1, 0.1, 0.1])]
    assert isinstance(masses, list)
    assert isinstance(box_sizes, list)
    assert len(masses) == len(box_sizes)
    num_boxes = len(masses)
    boxes = []
    boxes_geometry_id = []
    props = get_proximity_properties()
    for i in range(num_boxes):
        box_name = f"box{i}"
        box_idx = plant.AddModelInstance(box_name)
        box_body = plant.AddRigidBody(
            box_name, box_idx, SpatialInertia(
                masses[i], np.array([0, 0, 0]), UnitInertia(1, 1, 1)))
        boxes.append(box_body)
        box_shape = Box(box_sizes[i][0], box_sizes[i][1], box_sizes[i][2])
        box_geo = plant.RegisterCollisionGeometry(
            box_body, RigidTransform(), box_shape, f"{box_name}_box",
            props)
        boxes_geometry_id.append(box_geo)
        plant.RegisterVisualGeometry(
            box_body, RigidTransform(), box_shape, f"{box_name}_box",
            [0.8, 1.0, 0.0, 0.5])


def finalize_plant(plant: MultibodyPlant):
    # Choose contact model
    plant.set_contact_surface_representation(mesh_type)
    plant.set_contact_model(contact_model)
    plant.Finalize()


####################################
# Run Optimization
####################################
if optimize:
    # Create a system model (w/o visualizer) to do the optimization over
    builder_ = DiagramBuilder()
    plant_, scene_graph_ = AddMultibodyPlantSceneGraph(builder_, dt)

    arm_idx = add_robot(plant_)
    add_ground(plant_)
    ball_idx = add_ball(plant_)
    finalize_plant(plant_)

    builder_.ExportInput(plant_.get_actuation_input_port(), "control")
    system_ = builder_.Build()

    optEq = OptEq(system_, scene_graph_)
    optEq.SetInitialState(q_home)
    # q = optEq.SolveSampleIK()
    q = optEq.SolveContactProblem()
    x0 = np.hstack([q, np.zeros(7)])


####################################
# Run Simulation
####################################
if simulate:
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)
    plant.set_name("plant")
    scene_graph.set_name("scene_graph")
    arm_idx = add_robot(plant)
    add_ground(plant)
    add_ball(plant)
    finalize_plant(plant)

    controller_plant = MultibodyPlant(dt)
    controller_plant.set_name("controller_plant")
    add_robot(controller_plant)
    controller_plant.Finalize()
    # builder.AddSystem(controller_plant)

    target = ConstantVectorSource(x0)
    target_context = target.CreateDefaultContext()
    builder.AddSystem(target)
    controller = connect_pid_controller(
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
    plant.SetPositionsAndVelocities(plant_context, q_full0)

    # Simulate the system
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(True)
    simulator.Initialize()

    simulator.AdvanceTo(T)
