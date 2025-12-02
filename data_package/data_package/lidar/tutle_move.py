import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Turtlemovepub(Node):

	def __init__(self):
		super().__init__('turtle_movepub')
		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		timer.period = 2
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		msg = Twist()
		msg.linear