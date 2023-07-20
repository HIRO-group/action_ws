##
#
# A simple implementation of iterative LQR (iLQR) for discrete-time systems in Drake.
#
##

from pydrake.all import *
import time
import numpy as np
import pydrake.multibody.inverse_kinematics as ik


class OptEq():
    """
    Set up and solve a trajectory optimization problem of the form

    """

    def __init__(self, system, arm_idx):
        """
        Args:
            system:             Drake System describing the discrete-time dynamics
                                 x_{t+1} = f(x_t,u_t). Must be discrete-time.
        """
        assert system.IsDifferenceEquationSystem(
        )[0],  "must be a discrete-time system"

        # float-type copy of the system and context for linesearch.
        # Computations using this system are fast but don't support gradients
        self.system = system
        self.context = self.system.CreateDefaultContext()
        self.plant = system.GetSubsystemByName("plant")
        self.input_port = self.system.get_input_port(0)

        # Autodiff copy of the system for computing dynamics gradients
        self.system_ad = system.ToAutoDiffXd()
        self.context_ad = self.system_ad.CreateDefaultContext()
        self.plant_ad = self.system_ad.GetSubsystemByName("plant")
        self.input_port_ad = self.system_ad.get_input_port(0)
        context = self.plant_ad.GetMyMutableContextFromRoot(
            root_context=self.context_ad)
        init_ad = InitializeAutoDiff(np.zeros(7))
        self.plant_ad.get_actuation_input_port().FixValue(
            context, init_ad)

        print("plant->num_positions(): ", self.plant_ad.num_positions())
        print("plant->num_actuated_dofs(): ",
              self.plant_ad.num_actuated_dofs())

        self.context_ad.SetTimeStateAndParametersFrom(self.context)
        self.system_ad.FixInputPortsFrom(
            self.system, self.context, self.context_ad)

        # Define state and input sizes
        self.n = self.context.get_discrete_state_vector().size()
        print("self.n: ", self.n)

        # Initial and target states
        self.x0 = np.zeros(self.n)
        self.x_xom = np.zeros(self.n)

        self.panda_idx = self.plant.GetModelInstanceByName("panda")

    def SetInitialState(self, q0):
        """
        Fix the initial condition for the optimization.

        Args:
            x0: Vector containing the initial state of the system
        """
        self.q0 = q0

    def SolveContactProblem(self):
        prog = MathematicalProgram()
        q = prog.NewContinuousVariables(7)

        q_lower = np.array([-2.8973, -1.7628, -2.8973, -
                           3.0718, -2.8973, -0.0175, -2.8973])
        q_upper = np.array(
            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        prog.AddBoundingBoxConstraint(q_lower, q_upper, q)

        # Add basic cost. (This will be parsed into a QuadraticCost.)
        # prog.AddCost((q - self.q0).dot(q - self.q0))

        prog.AddConstraint(
            self.contact_constraint,
            lb=[0.0], ub=[0.01], vars=q)

        result = Solve(prog, initial_guess=self.q0)

        # result = Solve(prog)

        print(f"Success? {result.is_success()}")
        print(result.get_solution_result())
        q_sol = result.GetSolution(q)
        print("Solution: ", q_sol)

        return q_sol

    def contact_constraint(self, q):
        if q.dtype == float:
            print("float")
            plant = self.plant
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context)
        else:
            print("autodiff")
            plant = self.plant_ad
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context_ad)

        # Do forward kinematics.
        plant.SetPositions(context, self.panda_idx, q)

        contact_results = (
            plant.get_contact_results_output_port().Eval(context))

        num_pp = contact_results.num_point_pair_contacts()
        num_hydro = contact_results.num_hydroelastic_contacts()

        print("num point contacts = ", num_pp)
        print("num hydroelastic contacts = ", num_hydro)

        out = InitializeAutoDiff(np.ones((3, 3)))
        for i in range(num_hydro):
            contact = contact_results.hydroelastic_contact_info(i)
            centroid = contact.contact_surface().centroid()
            print("centroid: ", centroid)
            out = out.dot(centroid)
        print("out: ", out)

        return out

    def SolveSampleIK(self):
        prog = MathematicalProgram()
        q = prog.NewContinuousVariables(7)

        q_lower = np.array([-2.8973, -1.7628, -2.8973, -
                           3.0718, -2.8973, -0.0175, -2.8973])
        q_upper = np.array(
            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        prog.AddBoundingBoxConstraint(q_lower, q_upper, q)

        # Add basic cost. (This will be parsed into a QuadraticCost.)
        prog.AddCost((q - self.q0).dot(q - self.q0))

        prog.AddConstraint(
            self.link_7_distance_to_target_vector,
            lb=[0.0], ub=[0.01], vars=q)

        result = Solve(prog, initial_guess=self.q0)

        # result = Solve(prog)

        print(f"Success? {result.is_success()}")
        print(result.get_solution_result())
        q_sol = result.GetSolution(q)
        print("Solution: ", q_sol)

        return q_sol

    def link_7_distance_to_target_vector(self, q): return [
        self.link_7_distance_to_target(q)]

    def resolve_frame(self, plant, F):
        """Gets a frame from a plant whose scalar type may be different."""
        return plant.GetFrameByName(F.name(), F.model_instance())

    def link_7_distance_to_target(self, q):

        p_WT = [0.6, 0.6, 0.5]

        # Define some short aliases for frames.
        W = self.plant.world_frame()
        L7 = self.plant.GetFrameByName("panda_leftfinger", self.panda_idx)

        if q.dtype == float:
            # print("float")
            plant = self.plant
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context)
        else:
            # print("autodiff")
            plant = self.plant_ad
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context_ad)

        # Do forward kinematics.
        plant.SetPositions(context, self.panda_idx, q)

        X_WL7 = plant.CalcRelativeTransform(
            context, self.resolve_frame(plant, W), self.resolve_frame(plant, L7))
        p_TL7 = X_WL7.translation() - p_WT

        return p_TL7.dot(p_TL7)

    def SolveStaticEquilibriumProblem(self):
        """
        RuntimeError: Signed distance queries between shapes 'Convex' and 'Sphere'
        are not supported for scalar type drake::AutoDiffXd. See the documentation for
        QueryObject::ComputeSignedDistancePairwiseClosestPoints() for the full status of supported geometries.
        """

        (geometry_id1,) = self.plant_ad.GetCollisionGeometriesForBody(
            self.plant_ad.GetBodyByName("panda_hand"))
        (geometry_id2,) = self.plant_ad.GetCollisionGeometriesForBody(
            self.plant_ad.GetBodyByName("panda_hand"))

        ignored_collision_pairs = {(geometry_id1, geometry_id2)}

        context = self.plant_ad.GetMyMutableContextFromRoot(
            root_context=self.context_ad)

        dut = StaticEquilibriumProblem(
            plant=self.plant_ad, context=context, ignored_collision_pairs=ignored_collision_pairs)

        prog = dut.get_mutable_prog()
        q_lower = np.array([-2.8973, -1.7628, -2.8973, -
                           3.0718, -2.8973, -0.0175, -2.8973])
        q_upper = np.array(
            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        prog.AddBoundingBoxConstraint(
            q_lower, q_upper, dut.q_vars()[:7])

        ik.AddUnitQuaternionConstraintOnPlant(
            self.plant_ad, dut.q_vars(), prog)

        ball_quat = dut.q_vars()[7:11]
        prog.SetInitialGuess(
            ball_quat, np.array([0.5, 0.5, 0.5, 0.5]))

        # Add basic cost. (This will be parsed into a QuadraticCost.)
        # prog.AddCost((q - self.q0).dot(q - self.q0))
        result = Solve(prog)

        # result = Solve(prog)

        print(f"Success? {result.is_success()}")
        print(result.get_solution_result())
        q_sol = result.GetSolution(dut.q_vars())
        print("Solution: ", q_sol)
        contact_wrenches = dut.GetContactWrenchSolution(result)
        print(contact_wrenches)

        return q_sol
