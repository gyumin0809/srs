import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        # For Windows, the port will be like 'COM3'. For Linux, '/dev/ttyUSB0'
        # This needs to be changed depending on the environment.
        try:
            self.serial_port = serial.Serial('COM4', 115200, timeout=1) # Check your device manager for the correct port
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.02, self.read_imu_data)  # 50Hz

    def read_imu_data(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                return
            data = line.split(',')
            if len(data) < 10:
                return
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Assuming the order is: ax, ay, az, gx, gy, gz, ox, oy, oz, ow
            imu_msg.linear_acceleration.x = float(data[0])
            imu_msg.linear_acceleration.y = float(data[1])
            imu_msg.linear_acceleration.z = float(data[2])
            imu_msg.angular_velocity.x = float(data[3])
            imu_msg.angular_velocity.y = float(data[4])
            imu_msg.angular_velocity.z = float(data[5])
            imu_msg.orientation.x = float(data[6])
            imu_msg.orientation.y = float(data[7])
            imu_msg.orientation.z = float(data[8])
            imu_msg.orientation.w = float(data[9])
            
            self.publisher_.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
