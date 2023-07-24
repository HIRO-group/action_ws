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

    def __init__(self, system, scene_graph):
        """
        Args:
            system:             Drake System describing the discrete-time dynamics
                                 x_{t+1} = f(x_t,u_t). Must be discrete-time.
        """
        assert system.IsDifferenceEquationSystem(
        )[0],  "must be a discrete-time system"

        self.scene_graph = scene_graph
        self.inspector = scene_graph.model_inspector()

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
        self.m = self.input_port.size()
        print("self.m: ", self.m)

        # Initial and target states
        self.x0 = np.zeros(self.n)
        self.x_xom = np.zeros(self.n)

        self.panda_idx = self.plant.GetModelInstanceByName("panda")
        self.ball_idx = self.plant.GetModelInstanceByName("ball")

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
            lb=[0.004], ub=[0.005], vars=q)

        solver = SnoptSolver()
        assert (solver.available())
        # result = solver.Solve(prog)

        result = Solve(prog, initial_guess=self.q0)

        solver = result.get_solver_id()
        print("Solver: ", solver.name())

        print(f"Success? {result.is_success()}")
        print(result.get_solution_result())
        q_sol = result.GetSolution(q)
        print("Solution: ", q_sol)

        return q_sol

    def set_xu(self, q):
        obj_pos = np.array([0.2, 0., 0.05])
        obj_ori = np.array([0., 0., 0., 1])
        q_full0 = np.hstack(
            [q, obj_ori, obj_pos, np.zeros(9), np.zeros(4)])

        x = q_full0
        u = np.zeros(7)
        xu = np.hstack([x, u])
        return xu

    def contact_constraint(self, q):

        if q.dtype == float:
            print("float: ", q)
            xu = self.set_xu(q)
            x_ad = xu[:self.n]
            u_ad = xu[self.n:]
            self.context.SetDiscreteState(x_ad)
            x_start = self.context.get_discrete_state().get_vector().CopyToVector()
            state = self.context.get_discrete_state()
            self.input_port.FixValue(self.context, np.zeros(7))
            self.system.CalcForcedDiscreteVariableUpdate(
                self.context, state)
            x_next = state.get_vector().CopyToVector()

            x_ball_diff = np.array([x_start[13]-x_next[13]])
            # print(x_ball_diff)
            # out = next[13]

            plant = self.plant
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context)
            contact_results = (
                plant.get_contact_results_output_port().Eval(context))
            num_pp = contact_results.num_point_pair_contacts()
            num_hydro = contact_results.num_hydroelastic_contacts()
            print("num point contacts = ", num_pp)
            print("num hydroelastic contacts = ", num_hydro)

            total_area = 0
            for i in range(num_hydro):
                contact = contact_results.hydroelastic_contact_info(i)
                contact_surface = contact.contact_surface()
                centroid = contact_surface.centroid()
                # print(centroid)
                # print(ExtractValue(centroid))
                # print(ExtractGradient(centroid))

                geometry_id_m = contact_surface.id_M()
                geometry_id_n = contact_surface.id_N()

                frame_id_m = self.inspector.GetFrameId(geometry_id_m)
                frame_id_n = self.inspector.GetFrameId(geometry_id_n)

                print(plant.GetBodyFromFrameId(frame_id_m).name())
                print(plant.GetBodyFromFrameId(frame_id_n).name())

                total_area += contact_surface.total_area()
                print("total_area: ", total_area)

            return np.array([total_area])

        else:
            qval = ExtractValue(q)
            print("autodiff: ", qval[:, 0])
            xu = self.set_xu(qval[:, 0])
            xu_ad = InitializeAutoDiff(xu)
            x_ad = xu_ad[:self.n]
            u_ad = xu_ad[self.n:]
            self.context_ad.SetDiscreteState(x_ad)
            x_start = self.context_ad.get_discrete_state().get_vector().CopyToVector()
            state = self.context_ad.get_discrete_state()
            self.system_ad.CalcForcedDiscreteVariableUpdate(
                self.context_ad, state)
            x_next = state.get_vector().CopyToVector()
            x_ball_diff = x_start[13]-x_next[13]
            print("x_ball_diff: ", x_ball_diff)
            print("x_start[13]: ", x_start[13].value())
            print("x_next[13]: ", x_next[13].value())

            G = ExtractGradient(x_next)

            out = InitializeAutoDiff(
                np.array([x_start[13].value()]), np.array([G[13, 26]]))

            plant = self.plant_ad
            context = plant.GetMyMutableContextFromRoot(
                root_context=self.context_ad)
            contact_results = (
                plant.get_contact_results_output_port().Eval(context))
            num_pp = contact_results.num_point_pair_contacts()
            num_hydro = contact_results.num_hydroelastic_contacts()
            print("num point contacts = ", num_pp)
            print("num hydroelastic contacts = ", num_hydro)

            total_area = InitializeAutoDiff(np.array([0]), num_derivatives=34)
            for i in range(num_hydro):
                contact = contact_results.hydroelastic_contact_info(i)
                contact_surface = contact.contact_surface()
                centroid = contact_surface.centroid()
                # print(centroid)
                # print(ExtractValue(centroid))
                # print(ExtractGradient(centroid))

                geometry_id_m = contact_surface.id_M()
                geometry_id_n = contact_surface.id_N()

                frame_id_m = self.inspector.GetFrameId(geometry_id_m)
                frame_id_n = self.inspector.GetFrameId(geometry_id_n)

                print(plant.GetBodyFromFrameId(frame_id_m).name())
                print(plant.GetBodyFromFrameId(frame_id_n).name())

                total_area += contact_surface.total_area()
                print("total_area: ", total_area)

            return total_area

        # Do forward kinematics.

        # print("before")

        # print(ExtractValue(state_before))

        # print("after")
        # self.input_port_ad.FixValue(self.context_ad, u_ad)

        # plant.SetPositions(context, self.panda_idx, q)
        # ball_pos = InitializeAutoDiff(np.ones(7))
        # plant.SetPositions(context, self.ball_idx, ball_pos)

        # Compute the forward dynamics x_next = f(x,u)

        # contact_results = (
        #     plant.get_contact_results_output_port().Eval(context))
        # num_pp = contact_results.num_point_pair_contacts()
        # num_hydro = contact_results.num_hydroelastic_contacts()
        # print("num point contacts = ", num_pp)
        # print("num hydroelastic contacts = ", num_hydro)

        # frame_A = plant.GetFrameByName("ball")
        # print(frame_A.name())
        # vel = frame_A.CalcSpatialVelocityInWorld(context)
        # print("vel")
        # print(vel)

        # print(frame_A.CalcPoseInWorld(context))

        # out = InitializeAutoDiff(np.zeros((1, 3)), num_derivatives=7)
        # print(ExtractValue(out))
        # print(ExtractGradient(out))
        # for i in range(num_hydro):
        #     contact = contact_results.hydroelastic_contact_info(i)
        #     contact_surface = contact.contact_surface()
        #     centroid = contact_surface.centroid()
        #     # print(centroid)
        #     # print(ExtractValue(centroid))
        #     # print(ExtractGradient(centroid))

        #     geometry_id_m = contact_surface.id_M()
        #     geometry_id_n = contact_surface.id_N()

        #     frame_id_m = self.inspector.GetFrameId(geometry_id_m)
        #     frame_id_n = self.inspector.GetFrameId(geometry_id_n)

        #     print(plant.GetBodyFromFrameId(frame_id_m).name())
        #     print(plant.GetBodyFromFrameId(frame_id_n).name())

        #     force = contact.F_Ac_W()

        #     trans = force.translational()
        #     rot = force.rotational()

        #     print(ExtractValue(trans))
        #     print(ExtractGradient(trans))

        #     out += trans
        #     print("here")
        #     print(out)

        # print("value")
        # print(ExtractValue(out))
        # print("gradient")
        # print(ExtractGradient(out))
        # print("dot")

        # out = np.dot(out, out)
        # print(out)

        # return out

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
            # print(ExtractValue(q))
            # print(ExtractGradient(q))

        # Do forward kinematics.
        plant.SetPositions(context, self.panda_idx, q)

        X_WL7 = plant.CalcRelativeTransform(
            context, self.resolve_frame(plant, W), self.resolve_frame(plant, L7))
        p_TL7 = X_WL7.translation() - p_WT

        # print(p_TL7)
        # print(ExtractValue(p_TL7))
        # print(ExtractGradient(p_TL7))
        # print(p_TL7.dot(p_TL7))

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
