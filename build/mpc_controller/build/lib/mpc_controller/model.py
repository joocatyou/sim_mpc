# --- model.py ---
# 차량의 동역학 모델을 정의합니다.

from acados_template import AcadosModel
import casadi as ca

def create_differential_drive_model() -> AcadosModel:
    """
    미분 구동 로봇(Differential Drive Robot)의 5D Acados 모델을 생성합니다.
    상태: [x, y, psi, v, omega] (위치, 방향, 선속도, 각속도)
    제어: [a, alpha] (선가속도, 각가속도)
    """
    # --- 1. 모델 이름 및 파라미터 정의 ---
    model_name = 'differential_drive_robot'

    # 물리적 파라미터
    L = 0.5  # 바퀴 사이의 거리 (m)
    
    # 제어 입력 제한 (물리적 한계)
    v_max = 2.0     # 최대 선속도 (m/s)
    v_min = -1.0    # 최소 선속도 (m/s) - 후진 가능
    omega_max = 2.0 # 최대 각속도 (rad/s)
    omega_min = -2.0 # 최소 각속도 (rad/s)

    # --- 2. 심볼릭 변수 정의 ---
    # 상태 변수 (States) - 5D
    x = ca.SX.sym('x')          # x 위치
    y = ca.SX.sym('y')          # y 위치  
    psi = ca.SX.sym('psi')      # 헤딩 (방향)
    v = ca.SX.sym('v')          # 선속도
    omega = ca.SX.sym('omega')  # 각속도
    states = ca.vertcat(x, y, psi, v, omega)

    # 제어 입력 (Controls) - 2D
    a = ca.SX.sym('a')          # 선가속도
    alpha = ca.SX.sym('alpha')  # 각가속도
    controls = ca.vertcat(a, alpha)

    # 상태 변수의 시간 미분 (x_dot)
    x_dot = ca.SX.sym('x_dot', states.shape[0])

    # --- 3. 동역학(Dynamics) 정의 ---
    # 상태 방정식 (Continuous Dynamics)
    f_expl = ca.vertcat(
        v * ca.cos(psi),        # dx/dt = v * cos(psi)
        v * ca.sin(psi),        # dy/dt = v * sin(psi)
        omega,                  # dpsi/dt = omega
        a,                      # dv/dt = a
        alpha                   # domega/dt = alpha
    )

    f_impl = x_dot - f_expl

    # --- 4. Acados 모델 생성 및 반환 ---
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = x_dot
    model.u = controls
    model.name = model_name

    # 제어 입력 및 상태 제한을 모델에 저장 (참조용)
    model.v_max = v_max
    model.v_min = v_min
    model.omega_max = omega_max
    model.omega_min = omega_min

    # --- 5. 온라인 파라미터 정의 (경로 추종용) ---
    # p: 실시간으로 MPC에 전달될 값들
    # [x_ref, y_ref, psi_ref, v_ref, omega_ref] 5개의 참조값
    p = ca.SX.sym('p', 5)
    model.p = p

    return model
