import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
import matplotlib.pyplot as plt


class LMPCCAcados:
    def __init__(self):
        # MPC parameters
        self.N = 15  # prediction horizon
        self.dt = 0.2  # sampling time
        self.T_horizon = self.N * self.dt  # total horizon time
        
        # State and control dimensions
        self.nx = 6  # [x, y, theta, s, slack1, slack2]
        self.nu = 4  # [v, omega, slack_u1, slack_u2]
        
        # Cost function weights
        self.w_contour = 1.0      # contour error weight
        self.w_lag = 1.0          # lag error weight
        self.w_v = 1.5            # linear velocity weight
        self.w_omega = 0.1        # angular velocity weight
        self.w_slack = 10000.0    # slack variable weight
        
        # Reference path parameters (example: straight line)
        self.ref_path_coeffs = {
            'x_a': 0.0, 'x_b': 0.0, 'x_c': 1.0, 'x_d': 0.0,  # x = s
            'y_a': 0.0, 'y_b': 0.0, 'y_c': 0.0, 'y_d': 0.0   # y = 0
        }
        self.v_ref = 0.2  # reference velocity
        
        # Vehicle constraints
        self.v_max = 1.0
        self.v_min = -1.0
        self.omega_max = 1.0
        self.omega_min = -1.0
        
        # Setup ACADOS OCP
        self.ocp = self._setup_ocp()
        self.solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')
        
    def _setup_ocp(self):
        """Setup the ACADOS Optimal Control Problem"""
        ocp = AcadosOcp()
        
        # Model
        model = self._create_model()
        ocp.model = model
        
        # Dimensions and time horizon
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.T_horizon
        
        # Cost function
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        
        # Stage cost
        ocp.model.cost_y_expr = self._stage_cost_expr(model)
        ocp.cost.yref = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [e_c, e_l, v_error, omega, slack_u1, slack_u2]
        ocp.cost.W = np.diag([self.w_contour, self.w_lag, self.w_v, 
                             self.w_omega, self.w_slack, self.w_slack])
        
        # Terminal cost
        ocp.model.cost_y_expr_e = self._terminal_cost_expr(model)
        ocp.cost.yref_e = np.zeros(2)  # [e_c, e_l]
        ocp.cost.W_e = np.diag([self.w_contour, self.w_lag])
        
        # Constraints
        ocp.constraints.lbu = np.array([self.v_min, self.omega_min, -1e6, -1e6])
        ocp.constraints.ubu = np.array([self.v_max, self.omega_max, 1e6, 1e6])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        
        # Initial condition constraint
        ocp.constraints.x0 = np.zeros(self.nx)
        
        # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.nlp_solver_max_iter = 10
        ocp.solver_options.nlp_solver_tol_stat = 1e-3
        ocp.solver_options.nlp_solver_tol_eq = 1e-3
        ocp.solver_options.nlp_solver_tol_ineq = 1e-3
        
        return ocp
    
    def _create_model(self):
        """Create the vehicle dynamics model"""
        from acados_template import AcadosModel
        
        model = AcadosModel()
        model.name = 'lmpcc_model'
        
        # State variables
        x = ca.SX.sym('x')          # x position
        y = ca.SX.sym('y')          # y position
        theta = ca.SX.sym('theta')  # heading angle
        s = ca.SX.sym('s')          # path parameter
        slack1 = ca.SX.sym('slack1') # slack variable 1
        slack2 = ca.SX.sym('slack2') # slack variable 2
        
        states = ca.vertcat(x, y, theta, s, slack1, slack2)
        
        # Control inputs
        v = ca.SX.sym('v')           # linear velocity
        omega = ca.SX.sym('omega')   # angular velocity
        slack_u1 = ca.SX.sym('slack_u1')  # slack control 1
        slack_u2 = ca.SX.sym('slack_u2')  # slack control 2
        
        controls = ca.vertcat(v, omega, slack_u1, slack_u2)
        
        # No parameters needed - using fixed reference path
        
        # System dynamics
        x_dot = v * ca.cos(theta)
        y_dot = v * ca.sin(theta)
        theta_dot = omega
        s_dot = v
        slack1_dot = slack_u1
        slack2_dot = slack_u2
        
        f_expl = ca.vertcat(x_dot, y_dot, theta_dot, s_dot, slack1_dot, slack2_dot)
        
        # Model equations
        model.f_impl_expr = states - f_expl
        model.f_expl_expr = f_expl
        model.x = states
        model.xdot = ca.SX.sym('xdot', self.nx)
        model.u = controls
        
        return model
    
    def _stage_cost_expr(self, model):
        """Define stage cost expression"""
        x, y, theta, s, slack1, slack2 = ca.vertsplit(model.x)
        v, omega, slack_u1, slack_u2 = ca.vertsplit(model.u)
        
        # Use fixed path coefficients (straight line: x = s, y = 0)
        x_a, x_b, x_c, x_d = 0.0, 0.0, 1.0, 0.0
        y_a, y_b, y_c, y_d = 0.0, 0.0, 0.0, 0.0
        
        # Calculate reference path position and derivatives
        x_path = x_a * s**3 + x_b * s**2 + x_c * s + x_d
        y_path = y_a * s**3 + y_b * s**2 + y_c * s + y_d
        dx_path = 3 * x_a * s**2 + 2 * x_b * s + x_c
        dy_path = 3 * y_a * s**2 + 2 * y_b * s + y_c
        
        # Normalize gradients
        abs_grad = ca.sqrt(dx_path**2 + dy_path**2 + 1e-8)  # small epsilon for numerical stability
        dx_path_norm = dx_path / abs_grad
        dy_path_norm = dy_path / abs_grad
        
        # Calculate contour and lag errors
        e_c = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path)
        e_l = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path)
        
        # Return cost components - track velocity to reference
        v_error = v - self.v_ref
        return ca.vertcat(e_c, e_l, v_error, omega, slack_u1, slack_u2)
    
    def _terminal_cost_expr(self, model):
        """Define terminal cost expression"""
        x, y, theta, s, slack1, slack2 = ca.vertsplit(model.x)
        
        # Use fixed path coefficients (straight line: x = s, y = 0)
        x_a, x_b, x_c, x_d = 0.0, 0.0, 1.0, 0.0
        y_a, y_b, y_c, y_d = 0.0, 0.0, 0.0, 0.0
        
        # Calculate reference path position and derivatives
        x_path = x_a * s**3 + x_b * s**2 + x_c * s + x_d
        y_path = y_a * s**3 + y_b * s**2 + y_c * s + y_d
        dx_path = 3 * x_a * s**2 + 2 * x_b * s + x_c
        dy_path = 3 * y_a * s**2 + 2 * y_b * s + y_c
        
        # Normalize gradients
        abs_grad = ca.sqrt(dx_path**2 + dy_path**2 + 1e-8)
        dx_path_norm = dx_path / abs_grad
        dy_path_norm = dy_path / abs_grad
        
        # Calculate contour and lag errors
        e_c = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path)
        e_l = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path)
        
        return ca.vertcat(e_c, e_l)
    
    def solve(self, x0, reference_path=None):
       
        import time
        
        # Set initial condition
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        
        # No parameters to set - using fixed reference path
        
        # Solve
        start_time = time.time()
        status = self.solver.solve()
        solve_time = time.time() - start_time
        
        if status != 0:
            print(f"ACADOS solver failed with status {status}")
            return None, None, solve_time
        
        # Extract solution
        u_opt = self.solver.get(0, "u")
        
        # Get predicted trajectory
        x_pred = np.zeros((self.N + 1, self.nx))
        for i in range(self.N + 1):
            x_pred[i, :] = self.solver.get(i, "x")
        
        return u_opt, x_pred, solve_time
    
    def set_reference_path(self, path_coeffs):
        """Set reference path coefficients"""
        self.ref_path_coeffs = path_coeffs
    
    def set_weights(self, w_contour=None, w_lag=None, w_v=None, w_omega=None, w_slack=None):
        """Update cost function weights"""
        if w_contour is not None:
            self.w_contour = w_contour
        if w_lag is not None:
            self.w_lag = w_lag
        if w_v is not None:
            self.w_v = w_v
        if w_omega is not None:
            self.w_omega = w_omega
        if w_slack is not None:
            self.w_slack = w_slack
        
        # Update solver weights
        W = np.diag([self.w_contour, self.w_lag, self.w_v, 
                     self.w_omega, self.w_slack, self.w_slack])
        W_e = np.diag([self.w_contour, self.w_lag])
        
        for i in range(self.N):
            self.solver.cost_set(i, "W", W)
        self.solver.cost_set(self.N, "W", W_e)


def test_lmpcc():
    """Test the LMPCC controller"""
    
    # Initialize controller
    lmpcc = LMPCCAcados()
    
    # Define a simple reference path (straight line)
    ref_path = {
        'x_a': 0.0, 'x_b': 0.0, 'x_c': 1.0, 'x_d': 0.0,  # x = s
        'y_a': 0.0, 'y_b': 0.0, 'y_c': 0.0, 'y_d': 0.0   # y = 0
    }
    lmpcc.set_reference_path(ref_path)
    
    # Initial state [x, y, theta, s, slack1, slack2]
    x0 = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0])
    
    # Solve MPC
    u_opt, x_pred, solve_time = lmpcc.solve(x0)
    
    if u_opt is not None:
        print(f"Optimal control: v={u_opt[0]:.3f}, omega={u_opt[1]:.3f}")
        print(f"Solve time: {solve_time*1000:.2f} ms")
        
        # Plot results
        plt.figure(figsize=(12, 8))
        
        # Plot trajectory
        plt.subplot(2, 2, 1)
        plt.plot(x_pred[:, 0], x_pred[:, 1], 'b-o', label='Predicted trajectory')
        plt.plot(x_pred[:, 3], np.zeros_like(x_pred[:, 3]), 'r--', label='Reference path')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('Robot Trajectory')
        plt.legend()
        plt.grid(True)
        
        # Plot states over time
        time_vec = np.linspace(0, lmpcc.T_horizon, lmpcc.N + 1)
        
        plt.subplot(2, 2, 2)
        plt.plot(time_vec, x_pred[:, 0], label='x')
        plt.plot(time_vec, x_pred[:, 1], label='y')
        plt.plot(time_vec, x_pred[:, 2], label='theta')
        plt.xlabel('Time [s]')
        plt.ylabel('States')
        plt.title('State Evolution')
        plt.legend()
        plt.grid(True)
        
        # Plot controls
        plt.subplot(2, 2, 3)
        time_vec_u = np.linspace(0, lmpcc.T_horizon - lmpcc.dt, lmpcc.N)
        u_pred = np.zeros((lmpcc.N, lmpcc.nu))
        for i in range(lmpcc.N):
            u_pred[i, :] = lmpcc.solver.get(i, "u")
        
        plt.plot(time_vec_u, u_pred[:, 0], label='v')
        plt.plot(time_vec_u, u_pred[:, 1], label='omega')
        plt.xlabel('Time [s]')
        plt.ylabel('Controls')
        plt.title('Control Inputs')
        plt.legend()
        plt.grid(True)
        
        # Plot path parameter
        plt.subplot(2, 2, 4)
        plt.plot(time_vec, x_pred[:, 3], 'g-', label='s (path parameter)')
        plt.xlabel('Time [s]')
        plt.ylabel('Path parameter')
        plt.title('Path Parameter Evolution')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    else:
        print("Solver failed!")


if __name__ == "__main__":
    test_lmpcc()