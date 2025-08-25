#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import time

class CRC8Calculator:
    """CRC-8/MAXIM 계산기"""
    def __init__(self):
        self.polynomial = 0x31
        self.table = self._generate_table()
    
    def _generate_table(self):
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ self.polynomial
                else:
                    crc = crc << 1
                crc &= 0xFF
            table.append(crc)
        return table
    
    def calculate(self, data):
        crc = 0x00
        for byte in data:
            crc = self.table[crc ^ byte]
        return crc

class SimpleDDSM115Controller(Node):
    
    def __init__(self):
        super().__init__('simple_ddsm115_controller')
        
        # 파라미터 선언
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_motor_port', '/dev/ttyACM0'),
                ('right_motor_port', '/dev/ttyACM1'),
                ('baudrate', 115200),
                ('left_motor_id', 1),
                ('right_motor_id', 2),
                ('wheel_diameter', 0.1),          # 바퀴 직경 (m)
                ('wheel_base', 0.3),              # 바퀴 간격 (m)
                ('max_rpm', 330),                 # 최대 RPM
                ('cmd_vel_timeout', 1.0),         # cmd_vel 타임아웃 (s)
            ]
        )
        
        # 파라미터 읽기
        self.left_motor_port = self.get_parameter('left_motor_port').get_parameter_value().string_value
        self.right_motor_port = self.get_parameter('right_motor_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.left_motor_id = self.get_parameter('left_motor_id').get_parameter_value().integer_value
        self.right_motor_id = self.get_parameter('right_motor_id').get_parameter_value().integer_value
        self.wheel_diameter = self.get_parameter('wheel_diameter').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        
        # 물리적 파라미터
        self.wheel_radius = self.wheel_diameter / 2.0
        
        # CRC8 계산기
        self.crc8 = CRC8Calculator()
        
        # 현재 목표 속도
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.last_cmd_vel_time = time.time()
        
        # 시리얼 포트 초기화
        self.ser_left = None
        self.ser_right = None
        self.init_serial_ports()
        
        # cmd_vel 구독자
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 타임아웃 체크 타이머
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        
        # 상태 출력 타이머
        self.status_timer = self.create_timer(2.0, self.print_status)
        
        self.get_logger().info('Simple DDSM115 Controller Started')
        self.get_logger().info(f'Wheel diameter: {self.wheel_diameter}m, Wheel base: {self.wheel_base}m')
    
    def init_serial_ports(self):
        """시리얼 포트 초기화"""
        try:
            self.ser_left = serial.Serial(
                self.left_motor_port, 
                baudrate=self.baudrate, 
                timeout=0.1
            )
            self.ser_right = serial.Serial(
                self.right_motor_port, 
                baudrate=self.baudrate, 
                timeout=0.1
            )
            
            time.sleep(0.5)  # 포트 안정화
            
            # 속도 루프 모드로 전환
            self.set_speed_mode()
            
            self.get_logger().info(f'Serial ports initialized: {self.left_motor_port}, {self.right_motor_port}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to initialize serial ports: {e}')
            self.ser_left = None
            self.ser_right = None
    
    def set_speed_mode(self):
        """모터를 속도 루프 모드로 전환"""
        if not (self.ser_left and self.ser_right):
            return
            
        try:
            # 모드 전환 명령
            left_cmd = [self.left_motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]
            right_cmd = [self.right_motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]
            
            self.ser_left.write(bytes(left_cmd))
            self.ser_right.write(bytes(right_cmd))
            
            time.sleep(0.1)
            self.get_logger().info('Motors set to speed loop mode')
            
        except Exception as e:
            self.get_logger().error(f'Failed to set speed mode: {e}')
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 콜백 - 받은 값을 바로 적용"""
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z
        self.last_cmd_vel_time = time.time()
        
        # 차동구동 운동학으로 바퀴 속도 계산
        left_wheel_vel, right_wheel_vel = self.diff_drive_kinematics(
            self.target_linear_vel, self.target_angular_vel
        )
        
        # RPM으로 변환
        left_rpm = self.velocity_to_rpm(left_wheel_vel)
        right_rpm = self.velocity_to_rpm(right_wheel_vel)
        
        # 모터에 바로 전송
        self.send_motor_commands(left_rpm, right_rpm)
        
        self.get_logger().debug(
            f'CMD_VEL: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f} '
            f'→ RPM: L={left_rpm}, R={right_rpm}'
        )
    
    def check_timeout(self):
        """cmd_vel 타임아웃 확인"""
        if time.time() - self.last_cmd_vel_time > self.cmd_vel_timeout:
            if self.target_linear_vel != 0.0 or self.target_angular_vel != 0.0:
                self.get_logger().warn('CMD_VEL timeout - stopping motors')
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.send_motor_commands(0, 0)
    
    def diff_drive_kinematics(self, linear_vel, angular_vel):
        """차동구동 운동학"""
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base) / 2.0
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base) / 2.0
        return left_wheel_vel, right_wheel_vel
    
    def velocity_to_rpm(self, wheel_velocity):
        """바퀴 속도를 RPM으로 변환"""
        if abs(wheel_velocity) < 1e-6:
            return 0
        
        angular_velocity = wheel_velocity / self.wheel_radius
        rpm = angular_velocity * 60.0 / (2.0 * math.pi)
        
        # RPM 제한
        rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
        
        return int(round(rpm))
    
    def create_speed_command(self, motor_id, rpm):
        """속도 명령 패킷 생성"""
        # RPM을 16비트로 변환
        if rpm >= 0:
            rpm_16bit = rpm & 0xFFFF
        else:
            rpm_16bit = (rpm + 65536) & 0xFFFF
        
        high_byte = (rpm_16bit >> 8) & 0xFF
        low_byte = rpm_16bit & 0xFF
        
        # 기본 가속시간 설정
        accel_time = 10  # 1ms (0.1ms * 10)
        
        # 명령 패킷 구성
        cmd = [
            motor_id,      # DATA[0]: 모터 ID
            0x64,          # DATA[1]: 회전 명령
            high_byte,     # DATA[2]: 속도 상위 8비트
            low_byte,      # DATA[3]: 속도 하위 8비트
            0x00,          # DATA[4]: 0
            0x00,          # DATA[5]: 0
            accel_time,    # DATA[6]: 가속시간
            0x00,          # DATA[7]: 브레이크 해제
            0x00           # DATA[8]: 0
        ]
        
        # CRC8 계산
        crc8_value = self.crc8.calculate(cmd)
        cmd.append(crc8_value)
        
        return cmd
    
    def send_motor_commands(self, left_rpm, right_rpm):
        """모터 제어 명령 전송"""
        if not (self.ser_left and self.ser_right):
            return
        
        try:
            left_cmd = self.create_speed_command(self.left_motor_id, left_rpm)
            right_cmd = self.create_speed_command(self.right_motor_id, right_rpm)
            
            self.ser_left.write(bytes(left_cmd))
            self.ser_right.write(bytes(right_cmd))
            
        except Exception as e:
            self.get_logger().error(f'Failed to send motor commands: {e}')
    
    def print_status(self):
        """상태 출력"""
        left_vel, right_vel = self.diff_drive_kinematics(
            self.target_linear_vel, self.target_angular_vel
        )
        left_rpm = self.velocity_to_rpm(left_vel)
        right_rpm = self.velocity_to_rpm(right_vel)
        
        self.get_logger().info(
            f'Target: Linear={self.target_linear_vel:.3f}m/s, '
            f'Angular={self.target_angular_vel:.3f}rad/s → RPM: L={left_rpm}, R={right_rpm}'
        )
    
    def stop_motors(self):
        """모터 정지"""
        if self.ser_left and self.ser_right:
            try:
                self.send_motor_commands(0, 0)
                self.get_logger().info('Motors stopped')
            except Exception as e:
                self.get_logger().error(f'Failed to stop motors: {e}')
    
    def destroy_node(self):
        """노드 종료"""
        self.stop_motors()
        
        if self.ser_left:
            self.ser_left.close()
        if self.ser_right:
            self.ser_right.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleDDSM115Controller()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Keyboard interrupt received')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()