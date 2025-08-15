import rclpy
from rclpy.node import Node
import numpy as np
import socket
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header
import struct

class LidarSensorNode(Node):
    def __init__(self):
        super().__init__('lidar_sensor_node')
        self.get_logger().info("LiDAR Sensor Node Started")

        # Parameters
        self.declare_parameter('lidar.remote_ip', '192.168.1.77') # IP 주소를 컴퓨터의 IP로 변경
        self.declare_parameter('lidar.remote_port', 2368)
        self.declare_parameter('lidar.channel', 16)
        self.declare_parameter('lidar.frame_id', 'lidar_link')
        self.declare_parameter('lidar.reconnect_delay_sec', 5.0)

        self.remote_ip = self.get_parameter('lidar.remote_ip').get_parameter_value().string_value
        self.remote_port = self.get_parameter('lidar.remote_port').get_parameter_value().integer_value
        self.channel = self.get_parameter('lidar.channel').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('lidar.frame_id').get_parameter_value().string_value
        self.reconnect_delay_sec = self.get_parameter('lidar.reconnect_delay_sec').get_parameter_value().double_value

        # LiDAR specific configurations
        self.block_size = 1206
        self.max_len = 45
        self.vertical_angle_deg = np.array([[-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]])

        # Network setup
        self.socket = None # 소켓 초기화 제거
        self._connect_lidar() # _connect_lidar 호출

        # Publisher for LaserScan
        self.publisher_ = self.create_publisher(LaserScan, '/lidar/points', 10)
        self.timer = self.create_timer(0.1, self.publish_laser_scan)  # 10 Hz

    def _connect_lidar(self):
        if self.socket:
            self.socket.close()
            self.socket = None
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # SOF_TIMESTAMPING 옵션 추가
            # self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_TIMESTAMPING,
            #                        socket.SOF_TIMESTAMPING_RX_HARDWARE |
            #                        socket.SOF_TIMESTAMPING_RX_SOFTWARE |
            #                        socket.SOF_TIMESTAMPING_SOFTWARE)
            self.socket.settimeout(0.1) # 0.1초 타임아웃 설정
            self.socket.bind((self.remote_ip, self.remote_port)) # bind 사용
            self.get_logger().info(f"Successfully connected to LiDAR on {self.remote_ip}:{self.remote_port}")
            return True
        except socket.error as e:
            self.get_logger().error(f"Failed to connect to LiDAR on {self.remote_ip}:{self.remote_port}: {e}")
            self.socket = None
            return False

    def sph2cart(self, R, a):
        x = R * np.cos(np.deg2rad(self.vertical_angle_deg)) * np.sin(np.deg2rad(a))
        y = R * np.cos(np.deg2rad(self.vertical_angle_deg)) * np.cos(np.deg2rad(a))
        z = R * np.sin(np.deg2rad(self.vertical_angle_deg))
        return x.reshape([-1]), y.reshape([-1]), z.reshape([-1])

    def get_lidar_data_as_xyz(self):
        if self.socket is None:
            self.get_logger().warn("LiDAR socket is not connected. Attempting to reconnect...")
            if not self._connect_lidar():
                self.get_logger().error(f"Failed to reconnect to LiDAR. Retrying in {self.reconnect_delay_sec} seconds...")
                rclpy.spin_once(self, timeout_sec=self.reconnect_delay_sec) # Wait before retrying
                return None

        try:
            Buffer = b''
            for _ in range(self.max_len):
                UnitBlock, _ = self.socket.recvfrom(self.block_size)
                Buffer += UnitBlock[:1200]

            Buffer_np = np.frombuffer(Buffer, dtype=np.uint8).reshape([-1, 100])

            Azimuth = np.zeros((24 * self.max_len,))
            Azimuth[0::2] = (
                Buffer_np[:, 2].astype(np.float32)
                + 256 * Buffer_np[:, 3].astype(np.float32)
            )
            Azimuth[1::2] = Azimuth[0::2] + 20

            Distance = (
                Buffer_np[:, 4::3].astype(np.float32)
                + 256 * Buffer_np[:, 5::3].astype(np.float32)
            ) * 2

            Azimuth = Azimuth.reshape([-1, 1]) / 100
            Distance = Distance.reshape([-1, self.channel]) / 1000

            x, y, z = self.sph2cart(Distance, Azimuth)

            xyz = np.concatenate(
                [x.reshape([-1, 1]), y.reshape([-1, 1]), z.reshape([-1, 1])],
                axis=1
            ).astype(np.float32)

            return xyz
        except socket.error as e:
            self.get_logger().error(f"Socket error while receiving LiDAR data: {e}. Attempting to reconnect...")
            self.socket.close()
            self.socket = None
            rclpy.spin_once(self, timeout_sec=self.reconnect_delay_sec) # Wait before retrying
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to process LiDAR data: {e}")
            return None

    def create_laser_scan_msg(self, xyz_points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        min_angle = -np.pi
        max_angle = np.pi
        angle_increment = np.deg2rad(0.2)
        
        # Calculate angles and distances using vectorized NumPy operations
        x = xyz_points[:, 0]
        y = xyz_points[:, 1]
        
        angles = np.arctan2(y, x)
        distances = np.sqrt(x**2 + y**2)

        # Initialize ranges with infinity
        num_increments = int((max_angle - min_angle) / angle_increment)
        ranges = np.full(num_increments, float('inf'))

        # Map angles to indices and update ranges
        # Filter points within the valid angle range
        valid_indices = (angles >= min_angle) & (angles <= max_angle)
        
        # Calculate indices for valid points
        indices = ((angles[valid_indices] - min_angle) / angle_increment).astype(int)
        
        # Filter distances for valid points
        valid_distances = distances[valid_indices]

        # Update ranges array
        # Use a loop for updating ranges to handle multiple points mapping to the same index
        # and taking the minimum distance. This part is hard to vectorize directly for min.
        for i, dist in zip(indices, valid_distances):
            if 0 <= i < num_increments:
                ranges[i] = min(ranges[i], dist)

        laser_scan_msg = LaserScan(
            header=header,
            angle_min=min_angle,
            angle_max=max_angle,
            angle_increment=angle_increment,
            time_increment=0.0,
            scan_time=0.0,
            range_min=0.9,
            range_max=100.0,
            ranges=ranges.tolist(),
            intensities=[]
        )
        return laser_scan_msg

    def publish_laser_scan(self):
        xyz_points = self.get_lidar_data_as_xyz()
        if xyz_points is not None:
            laser_scan_msg = self.create_laser_scan_msg(xyz_points)
            self.publisher_.publish(laser_scan_msg)
            # self.get_logger().info(f"Published laser scan with {len(xyz_points)} points.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()