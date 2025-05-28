import crocoddyl
import pinocchio as pin
import numpy as np
import example_robot_data
from utils.load_indices import TalosIndices

class crocoddylOCP():
    
    def __init__(self):
        config = TalosIndices()
        self.qpos_indices, self.qvel_indices, self.ctrl_indices = config.load_mj_indices_dict()
        self.ocp_indices = config.load_ocp_indices_list()
        
        robot = example_robot_data.load("talos")
        self.model = robot.model
        self.q0 = self.model.referenceConfigurations["default"]
        self.x0 = np.concatenate([self.q0, np.zeros(self.model.nv)])

        rf_name = "right_sole_link"
        lf_name = "left_sole_link"
        lh_name = "gripper_left_joint"

        # Getting the frame ids
        self.rf_id = self.model.getFrameId(rf_name)
        self.lf_id = self.model.getFrameId(lf_name)
        self.lh_id = self.model.getFrameId(lh_name)

        self.state = crocoddyl.StateMultibody(self.model)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)

    def createActionModel(self, target=None):
        # Defining the multi-contact model (double-support contact)
        contacts = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
        lf_contact = crocoddyl.ContactModel6D(
            self.state,
            self.lf_id,
            pin.SE3.Identity(),
            pin.LOCAL_WORLD_ALIGNED,
            self.actuation.nu,
            np.array([0,0]),
        )
        rf_contact = crocoddyl.ContactModel6D(
            self.state,
            self.rf_id,
            pin.SE3.Identity(),
            pin.LOCAL_WORLD_ALIGNED,
            self.actuation.nu,
            np.array([0,0]),
        )
        contacts.addContact("lf_contact", lf_contact)
        contacts.addContact("rf_contact", rf_contact)
        
        # Defining the cost sum (cost manager)
        costs = crocoddyl.CostModelSum(self.state, self.actuation.nu)

        # Adding the hand-placement cost
        if target is not None:
            w_hand = np.array([1]*3 + [0.0001]*3)
            lh_Mref = pin.SE3(np.eye(3), target)
            activation_hand = crocoddyl.ActivationModelWeightedQuad(w_hand**2)
            lh_cost = crocoddyl.CostModelResidual(
                self.state,
                activation_hand,
                crocoddyl.ResidualModelFramePlacement(self.state, self.lh_id, lh_Mref, self.actuation.nu)
            )
            costs.addCost("lh_goal", lh_cost, 1e2)

        
        # Adding state and control regularization terms
        w_x = np.array([0] * 3 + [10.0] * 3 + [0.01] * (self.state.nv - 6) + [10] * self.state.nv)
        activation_xreg = crocoddyl.ActivationModelWeightedQuad(w_x**2)
        x_reg_cost = crocoddyl.CostModelResidual(
            self.state, activation_xreg, crocoddyl.ResidualModelState(self.state, self.x0, self.actuation.nu)
        )
        u_reg_cost = crocoddyl.CostModelResidual(
            self.state, crocoddyl.ResidualModelControl(self.state, self.actuation.nu)
        )
        costs.addCost("xReg", x_reg_cost, 1e-3)
        costs.addCost("uReg", u_reg_cost, 1e-4)

        # Adding the state limits penalization
        x_lb = np.concatenate([self.state.lb[1 : self.state.nv+1], self.state.lb[-self.state.nv :]])
        x_ub = np.concatenate([self.state.ub[1 : self.state.nv+1], self.state.ub[-self.state.nv :]])
        activation_xbounds = crocoddyl.ActivationModelQuadraticBarrier(
            crocoddyl.ActivationBounds(x_lb, x_ub)
        )
        x_bounds = crocoddyl.CostModelResidual(
            self.state,
            activation_xbounds,
            crocoddyl.ResidualModelState(self.state, 0*self.x0, self.actuation.nu),
        )
        costs.addCost("xBounds", x_bounds, 1.0)

        # Adding the friction cone penalization
        nsurf, mu = np.identity(3), 0.7
        cone = crocoddyl.FrictionCone(nsurf, mu, 4, False)
        activation_friction = crocoddyl.ActivationModelQuadraticBarrier(
            crocoddyl.ActivationBounds(cone.lb, cone.ub)
        )
        lf_friction = crocoddyl.CostModelResidual(
            self.state,
            activation_friction,
            crocoddyl.ResidualModelContactFrictionCone(self.state, self.lf_id, cone, self.actuation.nu),
        )
        rf_friction = crocoddyl.CostModelResidual(
            self.state,
            activation_friction,
            crocoddyl.ResidualModelContactFrictionCone(self.state, self.rf_id, cone, self.actuation.nu),
        )
        costs.addCost("lf_friction", lf_friction, 1e1)
        costs.addCost("rf_friction", rf_friction, 1e1)

        # Creating the action model
        dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(
            self.state, self.actuation, contacts, costs
        )

        self.dmodel = dmodel

        return dmodel
    

    def createSequence(self, dmodel, DT, N):
        runningModel = crocoddyl.IntegratedActionModelEuler(dmodel, DT)
        terminalModel = crocoddyl.IntegratedActionModelEuler(dmodel, 0.0)
        seq = [runningModel] * N + [terminalModel]
        return seq
    
    
    def createState(self, q):
        return np.concatenate([q, np.zeros(self.model.nv)])


    def createState(self, q, v):
        return np.concatenate([q, v])
    

    def createProblem(self, dmodel, x0, DT, N):
        seq = self.createSequence(dmodel, DT, N)
        self.problem = crocoddyl.ShootingProblem(x0, seq, seq[-1])
        self.fddp = crocoddyl.SolverFDDP(self.problem)
        self.fddp.setCallbacks([crocoddyl.CallbackVerbose()])

    
    def solveProblem(self):
        return (self.fddp.solve(), self.fddp.iter, self.fddp.cost)
    
    def doControl(self):
        return self.fddp.us[0]
    
    def getRiccatiGains(self):
        return self.fddp.K[0]