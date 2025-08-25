# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import os
from acados_template import AcadosOcpSolver
from .model import create_differential_drive_model

# ROS2 메시지 타입 임포트
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

# 쿼터니언-오일러 변환을 위한 유틸리티
from tf_transformations import euler_from_quaternion

class AcadosMpcController(Node):
    """
    공식 예제 스타일을 따른 개선된 ACADOS MPC 컨트롤러
    """
    def __init__(self):
        super().__init__('acados_mpc_controller')

        # 파라미터 선언
        self.declare_parameter('target_speed', 2.5)
        self.declare_parameter('prediction_horizon', 1.0)
        self.declare_parameter('N_horizon', 10)
        self.declare_parameter('control_frequency', 10.0)
        
        # 파라미터 값 가져오기
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.Tf = self.get_parameter('prediction_horizon').get_parameter_value().double_value
        self.N = self.get_parameter('N_horizon').get_parameter_value().integer_value
        control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.dt = 1.0 / control_freq

        self.get_logger().info(f"MPC 설정: Tf={self.Tf}, N={self.N}, dt={self.dt}")

        try:
            self.get_logger().info("ACADOS 솔버를 초기화합니다...")
            
            # 새로 생성된 솔버 로드
            self.model = create_differential_drive_model()
            
            # JSON 파일 경로 설정 (소스 디렉토리에서)
            src_dir = "/home/joo/autonomous_navigation_ws/src/mpc_controller/mpc_controller"
            json_file = os.path.join(src_dir, "acados_ocp_differential_drive_robot.json")
            
            # 솔버 생성 (generate=False, build=False로 기존 솔버 로드)
            self.solver = AcadosOcpSolver(None, json_file=json_file, generate=False, build=False)
            
            # 모델 차원 정보
            self.nx = self.model.x.rows()
            self.nu = self.model.u.rows()
            self.ny = self.nx + self.nu
            self.ny_e = self.nx
            self.np = self.model.p.rows()
            
            self.get_logger().info(f"솔버 초기화 완료: nx={self.nx}, nu={self.nu}, np={self.np}")
            
        except Exception as e:
            self.get_logger().error(f"ACADOS 솔버 초기화 실패: {e}")
            raise

        # ROS2 인터페이스 설정
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.predicted_path_pub = self.create_publisher(Path, '/predicted_path', 10)

        # 제어 루프 타이머
        self.timer = self.create_timer(self.dt, self.control_loop)

        # 상태 변수 초기화
        self.current_state = np.zeros(self.nx)
        self.global_path = None
        self.is_initialized = False
        
        # 솔버 통계
        self.solve_count = 0
        self.total_solve_time = 0.0
        self.max_solve_time = 0.0

        self.get_logger().info("ACADOS MPC 컨트롤러 초기화 완료")

    def odom_callback(self, msg: Odometry):

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        self.current_state = np.array([px, py, yaw, vx, wz])
        
        if not self.is_initialized:
            self.is_initialized = True
            # 초기 조건 설정 (공식 예제 스타일)
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)
            self.get_logger().info(f"초기 상태 설정 완료: {self.current_state}")

    def path_callback(self, msg: Path):
        """경로 콜백"""
        if len(msg.poses) > 0:
            self.global_path = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            self.get_logger().info(f"새로운 경로 수신: {len(self.global_path)} 포인트")

    def control_loop(self):
        if not self.is_initialized or self.global_path is None:
            self.get_logger().warn("초기화 대기 중...", throttle_duration_sec=2.0)
            return

        try:
            # 참조 궤적 생성 (5D 모델용)
            ref_trajectory = self._generate_reference_trajectory_5d()
            if ref_trajectory is None:
                self._publish_stop_command()
                return

            # 참조값 설정
            self._set_reference_trajectory_5d(ref_trajectory)
            
            # 파라미터 설정 (장애물 정보)
            # self._set_obstacle_parameters()
            
            # 현재 상태 제약 설정
            self.solver.set(0, "lbx", self.current_state)
            self.solver.set(0, "ubx", self.current_state)

            # 최적화 문제 해결
            import time
            solve_start = time.time()
            status = self.solver.solve()
            solve_time = time.time() - solve_start
            
            # 솔버 통계 업데이트
            self._update_solver_statistics(solve_time)
            
            if status != 0:
                self.get_logger().warn(f'솔버 상태: {status}', throttle_duration_sec=1.0)
                self._publish_stop_command()
                return

            # 제어 명령 추출 및 발행
            self._publish_control_command()
            
            # 예측 궤적 발행
            self._publish_predicted_path()

        except Exception as e:
            self.get_logger().error(f"제어 루프 오류: {e}")
            self._publish_stop_command()

    def _generate_reference_trajectory_5d(self):
        """참조 궤적 생성 - 공식 예제 스타일 개선"""
        if self.global_path is None or len(self.global_path) < 2:
            return None

        current_pos = self.current_state[:2]
        distances = np.linalg.norm(self.global_path - current_pos, axis=1)
        closest_idx = np.argmin(distances)

        # 참조 포인트 계산
        ref_points = np.zeros((self.N + 1, 2))
        path_idx = closest_idx
        ref_points[0, :] = self.global_path[path_idx]
        
        dist_increment = self.target_speed * self.dt
        current_ref_point = ref_points[0, :].copy()

        for i in range(1, self.N + 1):
            dist_to_travel = dist_increment
            
            while path_idx < len(self.global_path) - 1 and dist_to_travel > 1e-6:
                p_start = self.global_path[path_idx]
                p_end = self.global_path[path_idx + 1]
                segment_vec = p_end - p_start
                segment_len = np.linalg.norm(segment_vec)
                
                if segment_len < 1e-6:
                    path_idx += 1
                    continue
                
                dist_on_segment = np.linalg.norm(current_ref_point - p_start)
                remaining_segment = max(0, segment_len - dist_on_segment)
                
                if dist_to_travel <= remaining_segment:
                    ratio = dist_to_travel / segment_len
                    current_ref_point += ratio * segment_vec
                    break
                else:
                    dist_to_travel -= remaining_segment
                    path_idx += 1
                    if path_idx < len(self.global_path):
                        current_ref_point = self.global_path[path_idx].copy()
            
            ref_points[i, :] = current_ref_point

        # 5D 참조 궤적 구성: [x, y, psi, v, omega]
        ref_states = np.zeros((self.N + 1, 5))

        for i in range(self.N + 1):
            # 위치 참조
            ref_states[i, 0] = ref_points[i, 0]  # x_ref
            ref_states[i, 1] = ref_points[i, 1]  # y_ref
            
            # 헤딩 각도 계산
            if i < self.N:
                delta = ref_points[i+1] - ref_points[i]
                if np.linalg.norm(delta) > 1e-6:
                    ref_states[i, 2] = np.arctan2(delta[1], delta[0])
                else:
                    ref_states[i, 2] = ref_states[i-1, 2] if i > 0 else self.current_state[2]
            else:
                ref_states[i, 2] = ref_states[i-1, 2]
            
            # 속도 참조
            ref_states[i, 3] = self.target_speed  # v_ref
            ref_states[i, 4] = 0.0               # omega_ref

        return ref_states

    def _set_reference_trajectory_5d(self, ref_states):
        # 온라인 파라미터로 참조값 설정
        for k in range(self.N + 1):
            self.solver.set(k, "p", ref_states[k])  # [x_ref, y_ref, psi_ref, v_ref, omega_ref]


    def _publish_control_command(self):
        # 최적 제어 입력 추출
        u_opt = self.solver.get(0, "u")
        
        # 다음 상태를 이용한 속도 명령 (더 부드러운 제어)
        x_next = self.solver.get(1, "x")
        
        twist_msg = Twist()
        twist_msg.linear.x = float(x_next[3])   # v
        twist_msg.angular.z = float(x_next[4])  # omega
        
        # 제어 입력 제한 (모델과 일치)
        twist_msg.linear.x = np.clip(twist_msg.linear.x, -1.5, 3.5)
        twist_msg.angular.z = np.clip(twist_msg.angular.z, -3.0, 3.0)
        
        self.cmd_vel_pub.publish(twist_msg)

    def _publish_predicted_path(self):
        """예측 궤적 발행"""
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for k in range(self.N + 1):
            pose = PoseStamped()
            pose.header = path_msg.header
            x_pred = self.solver.get(k, "x")
            pose.pose.position.x = float(x_pred[0])
            pose.pose.position.y = float(x_pred[1])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        self.predicted_path_pub.publish(path_msg)

    def _publish_stop_command(self):
        """정지 명령 발행"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
    
    def _update_solver_statistics(self, solve_time):
        """솔버 통계 업데이트"""
        self.solve_count += 1
        self.total_solve_time += solve_time
        self.max_solve_time = max(self.max_solve_time, solve_time)
        
        if self.solve_count % 100 == 0:
            avg_time = self.total_solve_time / self.solve_count
            self.get_logger().info(
                f"솔버 통계 - 평균: {avg_time:.4f}s, 최대: {self.max_solve_time:.4f}s, "
                f"총 호출: {self.solve_count}회"
            )

    def destroy_node(self):
        """노드 소멸자"""
        if hasattr(self, 'solve_count') and self.solve_count > 0:
            avg_time = self.total_solve_time / self.solve_count
            self.get_logger().info(
                f"최종 솔버 통계 - 평균: {avg_time:.4f}s, 최대: {self.max_solve_time:.4f}s"
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    mpc_controller = AcadosMpcController()
    
    try:
        rclpy.spin(mpc_controller)
    except KeyboardInterrupt:
        pass
    finally:
        mpc_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
