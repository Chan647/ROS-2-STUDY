#!/usr/bin/env python3
#Lidar data preprocessing
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarAnalyzer(Node):
	def __init__(self):
		super().__init__('lidar_analyzer')
		self.subscription = self.create_subscription(LaserScan, "/set_data", self.listner_callback, 10)

	def listner_callback(self,msg):		
		#1. ROS data to Numpy
		ranges = np.array(msg.ranges)

		#2. preprocessing : inf data
		ranges = np.where(ranges == float('inf'), 3.5, ranges)

		#3. check front side 
		front_ranges = np.concatenate([ranges[:10], ranges[350:]])
		left_ranges = ranges[75:106]
		right_ranges = ranges[255:286]

		#4. calculate min and mean
		min_fdist = np.min(front_ranges)
		avg_fdist = np.mean(front_ranges)

		min_ldist = np.min(left_ranges)
		avg_ldist = np.mean(left_ranges)

		min_rdist = np.min(right_ranges)
		avg_rdist = np.mean(right_ranges)

		safe_dist = 0.4
		if min_fdist < safe_dist :
			self.get_logger().warn(f"dangerous! {min_fdist}m left!")
		else : 
			self.get_logger().info(f"safe forward ranges... {avg_fdist}m left ^^")

		if min_ldist < safe_dist :
			self.get_logger().warn(f"dangerous! {min_ldist}m left!")
		else : 
			self.get_logger().info(f"safe left ranges... {avg_ldist}m left ^^")

		if min_rdist < safe_dist :
			self.get_logger().warn(f"dangerous! {min_rdist}m left!")
		else : 
			self.get_logger().info(f"safe right ranges... {avg_rdist}m left ^^")



		def near(v, target, tol=0.05):
			return abs(v - target) < tol

		if near(min_fdist, 3.5)  and near(min_ldist, 3.5) and near(min_rdist, 3.5): 
			self.get_logger().info("forward...")

		elif near(min_fdist, 0.4) and near(min_ldist, 3.5) and near(min_rdist, 3.5):
			self.get_logger().info("turn right... ")

		elif near(min_fdist, 0,4) and near(min_ldist, 0.4):
			self.get_logger().info("backward and turn right...")

		elif near(min_fdist, 0.4) and near(min_rdist, 0.4):
			self.get_logger().info("backward and turn left...")


def main():
	rclpy.init()
	node = LidarAnalyzer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()