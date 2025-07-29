# -*- coding: utf-8 -*-

import numpy as np
import scipy.linalg
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat, sin, cos

def export_differential_drive_model():
    
    # 모델 구조체 생성
    model = AcadosModel()
    
    # 심볼릭 변수 정의
    x = SX.sym('x')         # 위치 x (m)
    y = SX.sym('y')         # 위치 y (m)
    yaw = SX.sym('yaw')     # 헤딩 각도 (rad)
    v = SX.sym('v')         # 선속도 (m/s)
    omega = SX.sym('omega') # 각속도 (rad/s)
    
    states = vertcat(x, y, yaw, v, omega)
    model.x = states
    
    # 제어 입력
    a = SX.sym('a')         # 선형 가속도 (m/s^2)
    alpha = SX.sym('alpha') # 각가속도 (rad/s^2)
    
    controls = vertcat(a, alpha)
    model.u = controls
    
    # 상태 미분
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    yaw_dot = SX.sym('yaw_dot')
    v_dot = SX.sym('v_dot')
    omega_dot = SX.sym('omega_dot')
    
    states_dot = vertcat(x_dot, y_dot, yaw_dot, v_dot, omega_dot)
    model.xdot = states_dot
    
    # 동역학 모델
    f_expl = vertcat(
        v * cos(yaw),
        v * sin(yaw),
        omega,
        a,
        alpha
    )
    
    model.f_impl_expr = states_dot - f_expl
    model.f_expl_expr = f_expl
    
    N_obs = 5
    model.p = SX.sym('p', 2 * N_obs)
    
    # 장애물 제약 조건
    robot_safety_radius = np.sqrt((0.42/2)**2 + (0.31/2)**2) + 0.30
    obstacle_constraints = []
    for i in range(N_obs):
        obs_pos = model.p[2*i:2*(i+1)]
        dist_sq_to_obs = (x - obs_pos[0])**2 + (y - obs_pos[1])**2
        obstacle_constraints.append(dist_sq_to_obs)
    
    model.con_h_expr = vertcat(*obstacle_constraints)
    
    # 모델 제약 및 파라미터 (공식 예제 스타일)
    model.v_min = -1.0
    model.v_max = 1.0
    model.omega_min = -1.0
    model.omega_max = 1.0
    model.a_min = -3.0
    model.a_max = 3.0
    model.alpha_min = -np.deg2rad(180)
    model.alpha_max = np.deg2rad(180)
    model.safety_radius_sq = robot_safety_radius**2
    model.N_obs = N_obs
    
    # 초기 조건
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    
    model.name = "diff_drive_robot"
    
    return model

def acados_settings_differential_drive(Tf, N):
    
    # OCP 생성
    ocp = AcadosOcp()
    
    # 모델 export
    model = export_differential_drive_model()
    
    # ACADOS 모델 설정
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.p = model.p
    model_ac.name = model.name
    model_ac.con_h_expr = model.con_h_expr
    
    ocp.model = model_ac
    
    # 차원 설정
    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx
    nh = model.con_h_expr.rows()
    
    ocp.dims.N = N
    
    # 비용 함수 설정 (공식 예제 스타일)
    Q = np.diag([10.0, 10.0, 5.0, 1.0, 5.0])  # 상태 가중치
    R = np.diag([0.5, 0.5])                    # 제어 가중치
    Qe = np.diag([20.0, 20.0, 10.0, 1.0, 5.0]) # 터미널 가중치
    
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    
    unscale = N / Tf
    
    ocp.cost.W = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe / unscale
    
    # 비용 함수 매핑 행렬
    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx
    
    Vu = np.zeros((ny, nu))
    Vu[nx:, :nu] = np.eye(nu)
    ocp.cost.Vu = Vu
    
    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e
    
    # 초기 참조값 설정
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)
    
    # 제약 조건 설정
    # 상태 제약
    ocp.constraints.lbx = np.array([model.v_min, model.omega_min])
    ocp.constraints.ubx = np.array([model.v_max, model.omega_max])
    ocp.constraints.idxbx = np.array([3, 4])  # v, omega 인덱스
    
    # 제어 입력 제약
    ocp.constraints.lbu = np.array([model.a_min, model.alpha_min])
    ocp.constraints.ubu = np.array([model.a_max, model.alpha_max])
    ocp.constraints.idxbu = np.array([0, 1])
    
    # 장애물 회피 제약
    ocp.constraints.lh = np.full(nh, model.safety_radius_sq)
    ocp.constraints.uh = np.full(nh, 1e9)
    
    # 초기 조건
    ocp.constraints.x0 = model.x0
    
    # 파라미터 초기값 (모든 장애물을 멀리 배치)
    ocp.parameter_values = np.full(2 * model.N_obs, 1000.0)
    
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4
    
    return ocp, model

def create_acados_solver_differential_drive(Tf=1.0, N=10, json_file="acados_ocp_diff_drive.json"):
    
    ocp, model = acados_settings_differential_drive(Tf, N)
    
    # 솔버 생성
    acados_solver = AcadosOcpSolver(ocp, json_file=json_file)
    
    return acados_solver, model, ocp
