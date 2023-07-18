##
#
# A simple implementation of iterative LQR (iLQR) for discrete-time systems in Drake.
#
##

from pydrake.all import *
import time
import numpy as np


class OptEq():
    """
    Set up and solve a trajectory optimization problem of the form

    """

    def __init__(self, system, input_port_index=0):
        """
        Args:
            system:             Drake System describing the discrete-time dynamics
                                 x_{t+1} = f(x_t,u_t). Must be discrete-time.
            input_port_index:   InputPortIndex for the control input u_t. Default is to
                                 use the first port.
        """
        assert system.IsDifferenceEquationSystem(
        )[0],  "must be a discrete-time system"

        # float-type copy of the system and context for linesearch.
        # Computations using this system are fast but don't support gradients
        self.system = system
        self.context = self.system.CreateDefaultContext()
        self.plant = system.GetSubsystemByName("plant")
        self.input_port = self.system.get_input_port(input_port_index)

        # Autodiff copy of the system for computing dynamics gradients
        self.system_ad = system.ToAutoDiffXd()
        self.context_ad = self.system_ad.CreateDefaultContext()
        self.plant_ad = self.system_ad.GetSubsystemByName("plant")
        self.input_port_ad = self.system_ad.get_input_port(input_port_index)

        # Define state and input sizes
        self.n = self.context.get_discrete_state_vector().size()
        self.m = self.input_port.size()

        # Initial and target states
        self.x0 = np.zeros(self.n)
        self.x_xom = np.zeros(self.n)

        self.p_WT = [0.6, 0.6, 0.5]

        # Define some short aliases for frames.
        self.W = self.plant.world_frame()
        self.panda_idx = self.plant.GetModelInstanceByName("panda")
        self.L7 = self.plant.GetFrameByName("panda_leftfinger", self.panda_idx)

    def SetInitialState(self, q0):
        """
        Fix the initial condition for the optimization.

        Args:
            x0: Vector containing the initial state of the system
        """
        self.q0 = q0

    def Solve(self):
        """
        Fix the initial condition for the optimization.

        Args:
            x0: Vector containing the initial state of the system
        """
        prog = MathematicalProgram()

        q = prog.NewContinuousVariables(7)
        # Define nominal configuration.

        q_lower = np.array([-2.8973, -1.7628, -2.8973, -
                           3.0718, -2.8973, -0.0175, -2.8973])
        q_upper = np.array(
            [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        prog.AddBoundingBoxConstraint(q_lower, q_upper, q)

        # Add basic cost. (This will be parsed into a QuadraticCost.)
        prog.AddCost((q - self.q0).dot(q - self.q0))

        def link_7_distance_to_target_vector(
            q): return [self.link_7_distance_to_target(q)]

        prog.AddConstraint(
            link_7_distance_to_target_vector,
            lb=[0.0], ub=[0.001], vars=q)

        result = Solve(prog, initial_guess=self.q0)

        print(f"Success? {result.is_success()}")
        print(result.get_solution_result())
        q_sol = result.GetSolution(q)
        print("Solution: ", q_sol)

        print(
            f"Initial distance: {self.link_7_distance_to_target(self.q0):.3f}")
        print(
            f"Solution distance: {self.link_7_distance_to_target(q_sol):.3f}")
        return q_sol

    def resolve_frame(self, plant, F):
        """Gets a frame from a plant whose scalar type may be different."""
        return plant.GetFrameByName(F.name(), F.model_instance())

    def link_7_distance_to_target(self, q):
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
            context, self.resolve_frame(plant, self.W), self.resolve_frame(plant, self.L7))
        p_TL7 = X_WL7.translation() - self.p_WT

        return p_TL7.dot(p_TL7)
