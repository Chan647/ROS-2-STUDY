#!/usr/bin/env python3
#Lidar data preprocessing
import rclpy
import os
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import csv

class LidarAnalyzer(Node):
	def __init__(self):
		super().__init__('lidar_analyzer')
		self.subscription = self.create_subscription(LaserScan, "/scan", self.listner_callback, 10)
	
	def listner_callback(self,msg):	
		self.state = "normal";	
		#1. ROS data to Numpy
		ranges = np.array(msg.ranges)

		#2. preprocessing : inf data
		ranges = np.where(ranges == float('inf'), 3.5, ranges)

		#3. check front side 
		front_ranges = np.concatenate([ranges[:10], ranges[350:]])
		back_ranges = ranges[170:189]
		left_ranges = ranges[75:106]
		right_ranges = ranges[255:286]

		#4. calculate min and mean
		min_fdist = np.min(front_ranges)
		avg_fdist = np.mean(front_ranges)

		min_ldist = np.min(left_ranges)
		avg_ldist = np.mean(left_ranges)

		min_rdist = np.min(right_ranges)
		avg_rdist = np.mean(right_ranges)

		min_bdist = np.min(back_ranges)
		avg_bdist = np.mean(back_ranges)

		safe_dist = 0.4
		if min_fdist < safe_dist :
			self.get_logger().warn(f"dangerous forward_side! {min_fdist}m left!")
			self.state = "forward_wall"
		else : 
			self.get_logger().info(f"safe forward ranges... {avg_fdist}m left ^^")

		if min_bdist < safe_dist :
			self.get_logger().warn(f"dangerous backward_side! {min_bdist}m left!")
			self.state = "backward_wall"
		else : 
			self.get_logger().info(f"safe backward ranges... {avg_bdist}m left ^^")

		if min_ldist < safe_dist :
			self.get_logger().warn(f"dangerous left_side! {min_ldist}m left!")
			self.state = "left_wall"
		else : 
			self.get_logger().info(f"safe left ranges... {avg_ldist}m left ^^")

		if min_rdist < safe_dist :	
			self.get_logger().warn(f"dangerous right_side! {min_rdist}m left!")
			self.state = "right_wall"
		else : 
			self.get_logger().info(f"safe right ranges... {avg_rdist}m left ^^")


		file_exists = os.path.exists("lidar_data.csv")

		with open('lidar_data.csv', mode='a') as file:
			writer = csv.writer(file)

			if not file_exists:
				header = [f"range_{i}" for i in range(360)] + ["situation"]
				writer.writerow(header)

			row = msg.ranges.tolist() + [self.state]
			writer.writerow(row)

		
def main():
	rclpy.init()
	node = LidarAnalyzer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()