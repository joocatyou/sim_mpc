import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
from sensor_msgs_py import point_cloud2

# TF2 related libraries
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration

# Point transformation libraries
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_geo

# QoS imports
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class DynamicCostmapNode(Node):
    def __init__(self):
        super().__init__('dynamic_costmap')
        self.target_frame = 'camera_link'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Method 1: Use default QoS (most compatible)
        # self.pc_sub = self.create_subscription(
        #     PointCloud2, '/cloud_obstacles', self.pointcloud_callback, 10)
        # self.costmap_pub = self.create_publisher(
        #     OccupancyGrid, '/local_costmap/costmap', 10)

        # Method 2: Explicit QoS profile for better compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Most navigation tools expect RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pc_sub = self.create_subscription(
            PointCloud2, '/cloud_obstacles', self.pointcloud_callback, qos_profile)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', qos_profile)

        # Method 3: Different QoS for input and output if needed
        # input_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,  # For sensor data
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )
        # output_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,    # For navigation
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )
        # self.pc_sub = self.create_subscription(
        #     PointCloud2, '/cloud_obstacles', self.pointcloud_callback, input_qos)
        # self.costmap_pub = self.create_publisher(
        #     OccupancyGrid, '/local_costmap/costmap', output_qos)

        self.width = 100
        self.height = 100
        self.resolution = 0.05
        self.origin_x = -self.width / 2.0 * self.resolution
        self.origin_y = -self.height / 2.0 * self.resolution

        self.get_logger().info('Dynamic Costmap Node started')

    def pointcloud_callback(self, cloud_msg):
        try:
            source_frame = cloud_msg.header.frame_id
            transformed_points = []

            # TF transformation logic
            if source_frame != self.target_frame:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame, source_frame, cloud_msg.header.stamp,
                        timeout=Duration(seconds=0.1))  # Reduced timeout
                except tf2_ros.TransformException as ex:
                    # Try with latest available transform
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.target_frame, source_frame, rclpy.time.Time(),
                            timeout=Duration(seconds=0.01))
                    except tf2_ros.TransformException:
                        self.get_logger().debug(f'Could not transform {source_frame} to {self.target_frame}: {ex}')
                        return

                # Transform points
                for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                    p_in = PointStamped()
                    p_in.header = cloud_msg.header
                    p_in.point.x = float(point[0])
                    p_in.point.y = float(point[1])
                    p_in.point.z = float(point[2])

                    p_out = tf2_geo.do_transform_point(p_in, transform)
                    transformed_points.append((p_out.point.x, p_out.point.y, p_out.point.z))

            else:
                # No transformation needed
                for point in point_cloud2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                    transformed_points.append((point[0], point[1], point[2]))

            # Generate costmap
            grid = OccupancyGrid()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = self.target_frame
            grid.info.resolution = self.resolution
            grid.info.width = self.width
            grid.info.height = self.height
            grid.info.origin.position.x = self.origin_x
            grid.info.origin.position.y = self.origin_y

            data = np.full(self.width * self.height, 0, dtype=np.int8)

            # Mark obstacles
            for p in transformed_points:
                x, y, z = p
                if abs(z) > 0.5:
                    continue

                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)

                if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                    index = grid_y * self.width + grid_x
                    data[index] = 100

            grid.data = data.tolist()
            self.costmap_pub.publish(grid)

        except Exception as e:
            self.get_logger().error(f'Error in costmap generation: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()