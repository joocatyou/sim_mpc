#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import tf2_ros
import tf2_py
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # TF 관련 추가
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 카메라 전방 영역 중심이지만 base_link 프레임으로 발행
        self.map_width = 100      # 전방 10m
        self.map_height = 100     # 좌우 10m
        self.map_resolution = 0.1  # 10cm per pixel
        self.map_origin_x = 0.0   # base_link 기준 원점
        self.map_origin_y = -5.0  # base_link 중심으로 좌우 5m
        
        # 카메라 FOV 제한 (더 넓은 각도로 부드러운 전환)
        self.camera_fov = 1.57   # 90도 (라디안) - 더 부드러운 동작
        
        # 레이저에서 base_link로의 오프셋 제거 (둘 다 base_link 기준)
        # 카메라 전방 영역만 사용하되 좌표계는 base_link 사용
        
        # Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        
        # Initialize costmap
        self.costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Timer for publishing costmap (더 느린 업데이트로 안정성 확보)
        self.timer = self.create_timer(0.2, self.publish_costmap)
        
        # 카메라 기준 cost map이므로 로봇 위치 추적 불필요
        # 모든 계산이 카메라 프레임에서 수행됨
        
        self.get_logger().info('Obstacle Detector initialized')
    
    def laser_callback(self, msg):
        """레이저 데이터를 카메라 프레임 기준으로 처리"""
        # 기존 데이터를 급격히 지우지 않고 천천히 감소 (부드러운 업데이트)
        self.costmap = (self.costmap * 0.7).astype(np.int8)  # 30% 감소
        
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            # Calculate angle
            angle = angle_min + i * angle_increment
            
            # 카메라 FOV 내부만 처리 (60도 범위)
            if abs(angle) > self.camera_fov / 2:
                continue
                
            # base_link 기준 장애물 위치 계산 (카메라 전방 영역)
            obs_x = range_val * math.cos(angle)
            obs_y = range_val * math.sin(angle)
            
            # 전방 영역만 필터링 (카메라 뒤쪽 무시)
            if obs_x <= 0:
                continue
                
            # Convert to grid coordinates
            grid_x = int(obs_x / self.map_resolution)
            grid_y = int((obs_y - self.map_origin_y) / self.map_resolution)
            
            # Check bounds
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                # Mark as obstacle
                self.costmap[grid_y, grid_x] = 100
                
                # Add inflation around obstacles
                self.inflate_obstacle(grid_x, grid_y)
    
    def inflate_obstacle(self, center_x, center_y):
        """Inflate obstacles for safer navigation"""
        inflation_radius = 3  # pixels (더 작은 반경으로 부드러운 동작)
        
        for dx in range(-inflation_radius, inflation_radius + 1):
            for dy in range(-inflation_radius, inflation_radius + 1):
                x = center_x + dx
                y = center_y + dy
                
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= inflation_radius:
                        # 더 부드러운 gradient cost
                        cost = max(0, 100 - int(distance * 20))  # 더 부드러운 감소
                        self.costmap[y, x] = max(self.costmap[y, x], cost)
    
    def publish_costmap(self):
        """Publish the costmap"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'base_link'
        
        # Cost map이 카메라 전방 영역을 중심으로 하되 base_link 프레임으로 발행
        # 이렇게 하면 navigation stack과 호환되며 끄덕임 현상 방지
        
        # Set map metadata
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Convert costmap to occupancy grid format
        grid_msg.data = self.costmap.flatten().tolist()
        
        self.costmap_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
