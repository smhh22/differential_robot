import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan

class LaserScanFrameIdConvertor(Node):
	def __init__(self):
		super().__init__('laser_scan_frame_id_convertor')
		self.get_logger().info("LaserScan FrameIdConverter starting.")
		self.publisher_ = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
		self.subscription_ = self.create_subscription(
			LaserScan, '/gz_lidar/scan', self.listener_callback, qos_profile_sensor_data
		)
		self.get_logger().info("LaserScan FrameIdConverter started. Listening to /gz_lidar/scan...")
	
	def listener_callback(self, msg):
		msg.header.frame_id = "rplidar"
		self.publisher_.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	laser_scan_frame_id_convertor = LaserScanFrameIdConvertor()
	rclpy.spin(laser_scan_frame_id_convertor)
	laser_scan_frame_id_convertor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
		
