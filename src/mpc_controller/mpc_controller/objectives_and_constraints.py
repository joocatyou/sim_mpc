# --- objectives_and_constraints.py ---
# MPC의 목표(비용 함수)와 제약조건을 정의합니다.

import casadi as ca
import numpy as np

def define_objective(model) -> dict:
    """
    5D 모델을 위한 경로 추종 비용 함수를 정의합니다.
    상태: [x, y, psi, v, omega]
    제어: [a, alpha]
    """
    # 상태와 제어 입력 변수 가져오기
    x = model.x  # [x, y, psi, v, omega]
    u = model.u  # [a, alpha]
    p = model.p  # [x_ref, y_ref, psi_ref, v_ref, omega_ref]

    # --- 가중치 행렬 (Weight Matrix) ---
    # 7x7 행렬: [x, y, psi, v, omega, a, alpha]
    W = np.diag([
        10.0,  # x 위치 오차에 대한 가중치
        10.0,  # y 위치 오차에 대한 가중치
        5.0,   # psi (헤딩) 오차에 대한 가중치
        8.0,   # v (선속도) 오차에 대한 가중치 - 증가! (빠른 속도 추종)
        2.0,   # omega (각속도) 오차에 대한 가중치 - 증가!
        0.05,  # a (선가속도) 사용량에 대한 가중치 - 감소! (더 적극적 가속)
        0.05   # alpha (각가속도) 사용량에 대한 가중치 - 감소!
    ])

    # 터미널 가중치 (마지막 상태에 대한 가중치)
    W_e = np.diag([
        20.0,  # 최종 x 위치 오차
        20.0,  # 최종 y 위치 오차
        10.0,  # 최종 헤딩 오차
        5.0,   # 최종 속도 오차
        2.0    # 최종 각속도 오차
    ])

    # --- 비용 함수 정의 (Least Squares Cost) ---
    # y: 비용을 계산할 변수들 [x, y, psi, v, omega, a, alpha]
    y_expr = ca.vertcat(x, u)

    # y_ref: 참조값 [x_ref, y_ref, psi_ref, v_ref, omega_ref, a_ref, alpha_ref]
    # 제어 입력의 참조값은 일반적으로 0 (부드러운 제어를 위함)
    y_ref = ca.vertcat(p, ca.SX.zeros(u.shape[0]))

    # 터미널 비용 (마지막 상태만)
    y_expr_e = x  # 상태만
    y_ref_e = p   # 참조 상태만

    return {
        'y_expr': y_expr,
        'y_ref': y_ref,
        'y_expr_e': y_expr_e,
        'y_ref_e': y_ref_e,
        'W': W,
        'W_e': W_e
    }

def define_constraints(model) -> list:
    """
    5D 모델을 위한 제약조건을 정의합니다.
    물리적 제한사항과 안전 제약을 포함할 수 있습니다.
    """
    # 현재는 비어있음 - 박스 제약조건은 generate_solver.py에서 처리
    # 비선형 제약조건이 필요한 경우 여기에 추가
    
    # 예시: 속도 제한 제약 (이미 박스 제약으로 처리됨)
    # return [model.x[3] - 0.1]  # v >= 0.1 (최소 속도)
    
    return []
