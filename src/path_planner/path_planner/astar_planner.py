#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10
        )
        
        # Internal variables
        self.costmap = None
        self.costmap_info = None
        self.start_pos = (0, 0)  # Robot start position in grid
        self.goal_pos = None
        
        self.get_logger().info('A* Planner initialized')
    
    def costmap_callback(self, msg):
        """Update costmap"""
        self.costmap_info = msg.info
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
        # Update robot position (assume robot is at origin for now)
        self.start_pos = self.world_to_grid(0.0, 0.0)
    
    def goal_callback(self, msg):
        """Handle goal position"""
        if self.costmap is None:
            self.get_logger().warn('No costmap received yet')
            return
        
        # Convert goal to grid coordinates
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.goal_pos = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(f'Goal received: ({goal_x}, {goal_y}) -> grid ({self.goal_pos[0]}, {self.goal_pos[1]})')
        
        # Plan path
        path = self.plan_path()
        if path:
            self.publish_path(path)
            self.publish_markers(path)
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        if self.costmap_info is None:
            return (0, 0)
        
        grid_x = int((x - self.costmap_info.origin.position.x) / self.costmap_info.resolution)
        grid_y = int((y - self.costmap_info.origin.position.y) / self.costmap_info.resolution)
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.costmap_info is None:
            return (0.0, 0.0)
        
        x = grid_x * self.costmap_info.resolution + self.costmap_info.origin.position.x
        y = grid_y * self.costmap_info.resolution + self.costmap_info.origin.position.y
        
        return (x, y)
    
    def is_valid_cell(self, x, y):
        """Check if cell is valid and free"""
        if (x < 0 or x >= self.costmap_info.width or 
            y < 0 or y >= self.costmap_info.height):
            return False
        
        return self.costmap[y, x] < 50  # Threshold for free space
    
    def get_neighbors(self, pos):
        """Get valid neighbors for A* search"""
        x, y = pos
        neighbors = []
        
        # 8-connected neighbors
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                if self.is_valid_cell(nx, ny):
                    cost = 1.0 if abs(dx) + abs(dy) == 1 else 1.414  # Diagonal cost
                    neighbors.append(((nx, ny), cost))
        
        return neighbors
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def plan_path(self):
        """A* path planning algorithm"""
        if self.goal_pos is None:
            return None
        
        start = self.start_pos
        goal = self.goal_pos
        
        # Check if start and goal are valid
        if not self.is_valid_cell(start[0], start[1]):
            self.get_logger().error('Start position is not valid')
            return None
        
        if not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().error('Goal position is not valid')
            return None
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'Path found with {len(path)} points')
                return path
            
            for neighbor, cost in self.get_neighbors(current):
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        self.get_logger().error('No path found')
        return None
    
    def publish_path(self, path):
        """Publish the planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        
        for grid_pos in path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'odom'
            
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1])
            pose.pose.position.x = world_pos[0]
            pose.pose.position.y = world_pos[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_markers(self, path):
        """Publish visualization markers for the path"""
        marker_array = MarkerArray()
        
        # Path line marker
        line_marker = Marker()
        line_marker.header.frame_id = 'odom'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for grid_pos in path:
            point = Point()
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1])
            point.x = world_pos[0]
            point.y = world_pos[1]
            point.z = 0.0
            line_marker.points.append(point)
        
        marker_array.markers.append(line_marker)
        
        # Path points markers
        for i, grid_pos in enumerate(path):
            point_marker = Marker()
            point_marker.header.frame_id = 'odom'
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = 'path_points'
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.scale.x = 0.1
            point_marker.scale.y = 0.1
            point_marker.scale.z = 0.1
            point_marker.color.r = 0.0
            point_marker.color.g = 1.0
            point_marker.color.b = 0.0
            point_marker.color.a = 1.0
            
            world_pos = self.grid_to_world(grid_pos[0], grid_pos[1])
            point_marker.pose.position.x = world_pos[0]
            point_marker.pose.position.y = world_pos[1]
            point_marker.pose.position.z = 0.0
            point_marker.pose.orientation.w = 1.0
            
            marker_array.markers.append(point_marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
