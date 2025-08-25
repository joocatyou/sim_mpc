#!/usr/bin/env python3
"""
ROS2 node for LMPCC controller using ACADOS
Compatible with ROS2 Humble
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import threading
from acados_lmpcc import LMPCCAcados


class LMPCCROSNode(Node):
    def __init__(self):
        super().__init__('lmpcc_acados_node')
        
        # Initialize LMPCC controller
        self.lmpcc = LMPCCAcados()
        
        # Current state [x, y, theta, s, slack1, slack2]
        self.current_state = np.zeros(6)
        self.state_received = False
        self.state_lock = threading.Lock()
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pred_traj_pub = self.create_publisher(Path, '/predicted_trajectory', 10)
        self.error_pub = self.create_publisher(Float64MultiArray, '/contour_error', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            Pose, '/robot_state', self.state_callback, 10
        )
        
        # Timer for control loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        # Reference path (can be updated via parameter server)
        self.setup_reference_path()
        
        self.get_logger().info("LMPCC ACADOS ROS2 node initialized")
        
    def setup_reference_path(self):
        """Setup reference path from ROS parameters"""
        try:
            # Declare parameters with default values
            self.declare_parameter('path.x_coeffs', [0.0, 0.0, 1.0, 0.0])
            self.declare_parameter('path.y_coeffs', [0.0, 0.0, 0.0, 0.0])
            self.declare_parameter('weights.contour', 1.0)
            self.declare_parameter('weights.lag', 1.0)
            self.declare_parameter('weights.velocity', 1.5)
            self.declare_parameter('weights.angular', 0.1)
            self.declare_parameter('weights.slack', 10000.0)
            
            # Get path coefficients from parameter server
            x_coeffs = self.get_parameter('path.x_coeffs').get_parameter_value().double_array_value
            y_coeffs = self.get_parameter('path.y_coeffs').get_parameter_value().double_array_value
            
            # Convert to list if needed
            if not x_coeffs:
                x_coeffs = [0.0, 0.0, 1.0, 0.0]
            if not y_coeffs:
                y_coeffs = [0.0, 0.0, 0.0, 0.0]
            
            ref_path = {
                'x_a': x_coeffs[0], 'x_b': x_coeffs[1], 
                'x_c': x_coeffs[2], 'x_d': x_coeffs[3],
                'y_a': y_coeffs[0], 'y_b': y_coeffs[1], 
                'y_c': y_coeffs[2], 'y_d': y_coeffs[3]
            }
            
            self.lmpcc.set_reference_path(ref_path)
            
            # Set weights
            w_contour = self.get_parameter('weights.contour').get_parameter_value().double_value
            w_lag = self.get_parameter('weights.lag').get_parameter_value().double_value
            w_v = self.get_parameter('weights.velocity').get_parameter_value().double_value
            w_omega = self.get_parameter('weights.angular').get_parameter_value().double_value
            w_slack = self.get_parameter('weights.slack').get_parameter_value().double_value
            
            self.lmpcc.set_weights(w_contour, w_lag, w_v, w_omega, w_slack)
            
            self.get_logger().info(f"Reference path set: x_coeffs={x_coeffs}, y_coeffs={y_coeffs}")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to load reference path from parameters: {e}")
            self.get_logger().info("Using default straight line path")
    
    def state_callback(self, msg):
        """Callback for robot state"""
        with self.state_lock:
            # Extract state from Pose message
            # Assuming Pose.position.z contains velocity and Pose.orientation.z contains heading
            self.current_state[0] = msg.position.x      # x
            self.current_state[1] = msg.position.y      # y  
            self.current_state[2] = msg.orientation.z   # theta
            self.current_state[3] = msg.position.z      # s (path parameter)
            # slack variables remain at 0
            self.current_state[4] = 0.0
            self.current_state[5] = 0.0
            
            self.state_received = True
    
    def control_callback(self):
        """Main control loop callback"""
        if not self.state_received:
            self.get_logger().warn("No state received yet", throttle_duration_sec=1.0)
            return
        
        try:
            with self.state_lock:
                current_state_copy = self.current_state.copy()
            
            # Solve MPC
            u_opt, x_pred, solve_time = self.lmpcc.solve(current_state_copy)
            
            if u_opt is not None:
                # Publish control command
                cmd_msg = Twist()
                cmd_msg.linear.x = float(u_opt[0])   # linear velocity
                cmd_msg.angular.z = float(u_opt[1])  # angular velocity
                self.cmd_pub.publish(cmd_msg)
                
                # Publish predicted trajectory
                self.publish_predicted_trajectory(x_pred)
                
                # Publish contour error
                self.publish_contour_error(current_state_copy)
                
                # Log performance
                self.get_logger().debug(f"MPC solved in {solve_time*1000:.2f} ms, "
                                      f"v={u_opt[0]:.3f}, omega={u_opt[1]:.3f}")
                
            else:
                self.get_logger().warn("MPC solver failed, publishing zero velocity")
                self.publish_zero_velocity()
                
        except Exception as e:
            self.get_logger().error(f"Error in control callback: {e}")
            self.publish_zero_velocity()
    
    def publish_predicted_trajectory(self, x_pred):
        """Publish predicted trajectory as Path message"""
        if x_pred is None:
            return
            
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"
            
            for i in range(len(x_pred)):
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = float(x_pred[i, 0])
                pose_stamped.pose.position.y = float(x_pred[i, 1])
                pose_stamped.pose.position.z = 0.0
                
                # Convert heading to quaternion (simplified)
                pose_stamped.pose.orientation.z = float(np.sin(x_pred[i, 2] / 2.0))
                pose_stamped.pose.orientation.w = float(np.cos(x_pred[i, 2] / 2.0))
                
                path_msg.poses.append(pose_stamped)
            
            self.pred_traj_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to publish predicted trajectory: {e}")
    
    def publish_contour_error(self, current_state):
        """Publish contour and lag errors"""
        try:
            # Calculate current contour and lag errors
            x, y, theta, s = current_state[:4]
            
            # Get reference path coefficients
            ref_path = self.lmpcc.ref_path_coeffs
            
            # Calculate reference position and derivatives
            x_path = ref_path['x_a'] * s**3 + ref_path['x_b'] * s**2 + ref_path['x_c'] * s + ref_path['x_d']
            y_path = ref_path['y_a'] * s**3 + ref_path['y_b'] * s**2 + ref_path['y_c'] * s + ref_path['y_d']
            dx_path = 3 * ref_path['x_a'] * s**2 + 2 * ref_path['x_b'] * s + ref_path['x_c']
            dy_path = 3 * ref_path['y_a'] * s**2 + 2 * ref_path['y_b'] * s + ref_path['y_c']
            
            # Normalize gradients
            abs_grad = np.sqrt(dx_path**2 + dy_path**2 + 1e-8)
            dx_path_norm = dx_path / abs_grad
            dy_path_norm = dy_path / abs_grad
            
            # Calculate errors
            e_c = dy_path_norm * (x - x_path) - dx_path_norm * (y - y_path)
            e_l = -dx_path_norm * (x - x_path) - dy_path_norm * (y - y_path)
            
            # Publish errors
            error_msg = Float64MultiArray()
            error_msg.data = [float(e_c), float(e_l)]
            self.error_pub.publish(error_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to calculate/publish contour error: {e}")
    
    def publish_zero_velocity(self):
        """Publish zero velocity command"""
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    """Main function for ROS2 node"""
    rclpy.init(args=args)
    
    try:
        node = LMPCCROSNode()
        
        # Use MultiThreadedExecutor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Error starting LMPCC node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()