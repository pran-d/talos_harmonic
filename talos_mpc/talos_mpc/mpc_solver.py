import crocoddyl
import pinocchio as pin
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class MPCSolver():
    def __init__(self, com_position, logger):
        self.resource_path = get_package_share_directory("talos_harmonic")
        self.urdf_path = Path(self.resource_path) / "urdf" / "talos_full_fake_grippers.urdf"
        self.srdf_path = Path(self.resource_path) / "urdf" / "talos.srdf"
        
        robot = pin.robot_wrapper.RobotWrapper.BuildFromURDF(
            str(self.urdf_path),
            [self.resource_path],
            pin.JointModelFreeFlyer()
        )

        self.com_position = com_position
        
        self.rmodel = robot.model
        pin.loadRotorParameters(self.rmodel, self.srdf_path)
        self.rmodel.armature = np.multiply(
            self.rmodel.rotorInertia.flat, np.square(self.rmodel.rotorGearRatio.flat)
        )
        pin.loadReferenceConfigurations(self.rmodel, self.srdf_path, False)
        q0 = pin.neutral(self.rmodel)
        q0 = self.rmodel.referenceConfigurations["half_sitting"].copy()
        q0 = pin.normalize(self.rmodel, q0)
        self.x0 = np.concatenate([q0, np.zeros(self.rmodel.nv)])

        # Getting the frame ids
        self.ee_ids = {}
        self.ee_ids["rf"] = self.rmodel.getFrameId("right_sole_link")
        self.ee_ids["lf"] = self.rmodel.getFrameId("left_sole_link")
        # self.ee_ids["rh"] = self.rmodel.getFrameId("gripper_right_joint")
        # self.ee_ids["lh"] = self.rmodel.getFrameId("gripper_left_joint")

        self.state = crocoddyl.StateMultibody(self.rmodel)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)

    def createProblemFromInitial(self, x0, DT, N, target=None):
        # Defining the multi-contact model (double-support contact)
        self.contacts = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
        lf_contact = crocoddyl.ContactModel6D(
            self.state,
            self.ee_ids["lf"],
            pin.SE3.Identity(),
            pin.LOCAL_WORLD_ALIGNED,
            self.actuation.nu,
            np.array([0., 4.]),
        )
        rf_contact = crocoddyl.ContactModel6D(
            self.state,
            self.ee_ids["rf"],
            pin.SE3.Identity(),
            pin.LOCAL_WORLD_ALIGNED,
            self.actuation.nu,
            np.array([0., 4.]),
        )
        self.contacts.addContact("lf_contact", lf_contact)
        self.contacts.addContact("rf_contact", rf_contact)
        
        # Defining the cost sum (cost manager)
        self.costs = crocoddyl.CostModelSum(self.state, self.actuation.nu)

        # Adding the hand-placement cost
        if target is not None:
            w_hand = np.array([1]*3 + [0.0001]*3)
            lh_Mref = pin.SE3(np.eye(3), target)
            activation_hand = crocoddyl.ActivationModelWeightedQuad(w_hand**2)
            lh_cost = crocoddyl.CostModelResidual(
                self.state,
                activation_hand,
                crocoddyl.ResidualModelFramePlacement(self.state, self.ee_ids["lh"], lh_Mref, self.actuation.nu)
            )
            self.costs.addCost("lh_goal", lh_cost, 1e2)

        
        # Adding state and control regularization terms
        w_x = np.array(
            [500] * 3 + 
            [1000] * 3 + 
            [500, 500, 500, 500, 1000, 1000] * 2 +
            [100, 200] + 
            [100, 100, 100, 100, 1, 1, 1] * 2 +
            [500, 500] + 
            [1] * 6 +
            [1] * 12 + 
            [10, 10] + 
            [10] * 14 +
            [100, 100]
        )
        activation_xreg = crocoddyl.ActivationModelWeightedQuad(w_x**2)
        x_reg_cost = crocoddyl.CostModelResidual(
            self.state, activation_xreg, crocoddyl.ResidualModelState(self.state, self.x0, self.actuation.nu)
        )
        w_u = np.array(
            [1] * 30
        )
        activation_ureg = crocoddyl.ActivationModelWeightedQuad(w_u**2)
        u_reg_cost = crocoddyl.CostModelResidual(
            self.state, activation_ureg, crocoddyl.ResidualModelControl(self.state, self.actuation.nu)
        )
        self.costs.addCost("xReg", x_reg_cost, 2e-2)
        self.costs.addCost("uReg", u_reg_cost, 1e-3)

        # Adding the state limits penalization
        x_lb = np.concatenate([self.state.lb[1 : self.state.nv+1], self.state.lb[-self.state.nv :]])
        x_ub = np.concatenate([self.state.ub[1 : self.state.nv+1], self.state.ub[-self.state.nv :]])
        activation_xbounds = crocoddyl.ActivationModelQuadraticBarrier(
            crocoddyl.ActivationBounds(x_lb, x_ub)
        )
        x_bounds = crocoddyl.CostModelResidual(
            self.state,
            activation_xbounds,
            crocoddyl.ResidualModelState(self.state, self.actuation.nu),
        )
        self.costs.addCost("xBounds", x_bounds, 1e3)

        # Adding the friction cone penalization
        nsurf, mu = np.identity(3), 0.7
        cone = crocoddyl.FrictionCone(nsurf, mu, 4, False)
        activation_friction = crocoddyl.ActivationModelQuadraticBarrier(
            crocoddyl.ActivationBounds(cone.lb, cone.ub)
        )
        lf_friction = crocoddyl.CostModelResidual(
            self.state,
            activation_friction,
            crocoddyl.ResidualModelContactFrictionCone(self.state, self.ee_ids["lf"], cone, self.actuation.nu),
        )
        rf_friction = crocoddyl.CostModelResidual(
            self.state,
            activation_friction,
            crocoddyl.ResidualModelContactFrictionCone(self.state, self.ee_ids["rf"], cone, self.actuation.nu),
        )
        self.costs.addCost("lf_friction", lf_friction, 1e1)
        self.costs.addCost("rf_friction", rf_friction, 1e1)

        # Adding the feet wrench cost
        wrenchCone_LF = crocoddyl.WrenchCone(
            np.identity(3), 0.3, np.array([0.1, 0.05]), 4, True, 300, 1200
        )
        wrenchCone_RF = crocoddyl.WrenchCone(
            np.identity(3), 0.3, np.array([0.1, 0.05]), 4, True, 300, 1200
        )
        residual_LF_wrench = crocoddyl.ResidualModelContactWrenchCone(
            self.state,
            self.ee_ids["lf"],
            wrenchCone_LF,
            self.actuation.nu,
        )
        residual_RF_wrench = crocoddyl.ResidualModelContactWrenchCone(
            self.state,
            self.ee_ids["rf"],
            wrenchCone_RF,
            self.actuation.nu,
        )
        wrenchModel_LF = crocoddyl.CostModelResidual(
            self.state,
            residual_LF_wrench
        )
        wrenchModel_RF = crocoddyl.CostModelResidual(
            self.state,
            residual_RF_wrench
        )
        self.costs.addCost("wrench_LF", wrenchModel_LF, 0)
        self.costs.addCost("wrench_RF", wrenchModel_RF, 0)

        # Adding COM position cost
        com_position_cost = crocoddyl.CostModelResidual(
            self.state,
            crocoddyl.ResidualModelCoMPosition(self.state, self.com_position, self.actuation.nu)
        )
        self.costs.addCost("comPosition", com_position_cost, 500)

        # Creating the action model
        dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(
            self.state, self.actuation, self.contacts, self.costs
        )

        runningModel = crocoddyl.IntegratedActionModelEuler(dmodel, DT)
        self.seq = [runningModel] * N
        self.problem = crocoddyl.ShootingProblem(x0, self.seq, self.seq[-1])
        self.fddp = crocoddyl.SolverFDDP(self.problem)


    def updateProblem(self, x0):
        self.problem.circularAppend(
            self.problem.runningModels[0],
            self.problem.runningDatas[0]
        )
        self.problem.x0 = x0
        return True


    def createState(self, q, v=None):
        if v is None:
            v = np.zeros((self.rmodel.nv,1))
        return np.concatenate([q, v])


    def solveProblem(self, x0, maxiter=100):
        # warm start for states
        warm_xs = self.getStateSequence()
        del warm_xs[0]
        warm_xs[0] = x0
        warm_xs.append(warm_xs[-1])

        # warm start for control inputs
        warm_us = self.getControlSequence()
        del warm_us[0]
        warm_us.append(warm_us[-1])

        return (self.fddp.solve(warm_xs, warm_us, maxiter), self.fddp.iter, self.fddp.cost)

    def getControlSequence(self):
        return self.fddp.us.copy()

    def getStateSequence(self):
        return self.fddp.xs.copy()

    def getRiccatiGainSequence(self):
        return self.fddp.K.copy()