# --- generate_solver.py ---
# 위에서 정의한 모델과 목표/제약조건을 바탕으로 Acados MPC 솔버를 생성합니다.

import os
from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca

from .model import create_differential_drive_model
from .objectives_and_constraints import define_objective, define_constraints

def main():
    # --- 1. MPC 파라미터 설정 ---
    N = 20  # 예측 구간의 길이 (Horizon length)
    Tf = 2.0  # 예측 총 시간 (초). T = Tf/N = 0.1초 (10Hz)

    # 제어 입력의 경계값 (가속도)
    A_MAX = 4.0    # 최대 선가속도 (m/s^2) - 증가!
    A_MIN = -4.0   # 최소 선가속도 (m/s^2) - 증가!
    ALPHA_MAX = 4.0  # 최대 각가속도 (rad/s^2) - 증가!
    ALPHA_MIN = -4.0 # 최소 각가속도 (rad/s^2) - 증가!

    # --- 2. Acados OCP(Optimal Control Problem) 객체 생성 ---
    ocp = AcadosOcp()

    # --- 3. 모델 설정 ---
    model = create_differential_drive_model()
    ocp.model = model

    # --- 4. 차원(Dimensions) 설정 ---
    ocp.solver_options.N_horizon = N

    # --- 5. 비용(Cost) 함수 설정 ---
    ocp.cost.cost_type = 'LINEAR_LS'  # 선형 최소제곱 비용함수
    ocp.cost.cost_type_e = 'LINEAR_LS' # 터미널 비용함수

    objective_params = define_objective(model)
    
    # 가중치 행렬 설정
    ocp.cost.W = objective_params['W']
    ocp.cost.W_e = objective_params['W_e']

    # 참조값 초기화
    ocp.cost.yref = np.zeros(objective_params['y_expr'].shape)
    ocp.cost.yref_e = np.zeros(model.x.shape[0])

    # 비용 함수 표현식 설정
    ocp.model.cost_y_expr = objective_params['y_expr']
    ocp.model.cost_y_expr_e = objective_params['y_expr_e']

    # Jacobian 행렬 설정 (상태와 제어에 대한)
    ocp.cost.Vx = np.zeros((objective_params['y_expr'].shape[0], model.x.shape[0]))
    ocp.cost.Vx[:model.x.shape[0], :model.x.shape[0]] = np.eye(model.x.shape[0])

    ocp.cost.Vu = np.zeros((objective_params['y_expr'].shape[0], model.u.shape[0]))
    ocp.cost.Vu[model.x.shape[0]:, :] = np.eye(model.u.shape[0])

    ocp.cost.Vx_e = np.eye(model.x.shape[0])

    # --- 6. 제약조건(Constraints) 설정 ---
    # 제어 입력(u)에 대한 박스 제약 [a, alpha]
    ocp.constraints.lbu = np.array([A_MIN, ALPHA_MIN])
    ocp.constraints.ubu = np.array([A_MAX, ALPHA_MAX])
    ocp.constraints.idxbu = np.array([0, 1]) # u[0]=a, u[1]=alpha에 제약

    # 상태 변수(x)에 대한 제약 [v, omega]만 제한
    # 속도와 각속도에 물리적 제한 설정
    ocp.constraints.lbx = np.array([model.v_min, model.omega_min])
    ocp.constraints.ubx = np.array([model.v_max, model.omega_max])
    ocp.constraints.idxbx = np.array([3, 4]) # v, omega에만 제약 (x, y, psi는 자유)

    # 비선형 제약조건 (향후 확장)
    ocp.model.con_h_expr = ca.vertcat(*define_constraints(model))
    ocp.constraints.lh = np.zeros(ocp.model.con_h_expr.shape) # h(x,u) >= 0
    ocp.constraints.uh = np.full(ocp.model.con_h_expr.shape, 1e9) # h(x,u) <= inf

    # 초기 상태(x0)는 파라미터로 설정 (실시간으로 현재 로봇 상태를 받음)
    ocp.constraints.x0 = np.zeros(model.x.shape)

    # --- 7. 온라인 파라미터(p) 초기값 설정 ---
    # [x_ref, y_ref, psi_ref, v_ref, omega_ref]
    ocp.parameter_values = np.zeros(model.p.shape)

    # --- 8. 솔버 옵션 설정 ---
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # Real-Time Iteration
    ocp.solver_options.tf = Tf

    # --- 9. 솔버 생성 ---
    # 생성된 파일들이 저장될 디렉토리
    code_gen_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "c_generated_code")
    if not os.path.exists(code_gen_dir):
        os.makedirs(code_gen_dir)

    solver_json = 'acados_ocp_' + model.name + '.json'
    AcadosOcpSolver(ocp, json_file=solver_json, build=True, generate=True)

    print(f"성공적으로 '{model.name}' 모델의 Acados 솔버를 생성했습니다.")
    print(f"생성된 파일은 '{code_gen_dir}' 폴더에서 확인할 수 있습니다.")


if __name__ == "__main__":
    main()
