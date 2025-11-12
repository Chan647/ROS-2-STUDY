import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimPublisher(Node):
	
	def __init__(self):
		super().__init__('turtlesim_publisher')
		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

	

def main(args=None):
	rp.init(args=args)
	turtlesim_publisher = TurtlesimPublisher()

	msg = Twist()

	while rp.ok():
		key_input = input("key :")

		if key_input == 'w':
			msg.linear.x = 2.0
			msg.angular.z = 0.0

		elif key_input == 'a':
			msg.angular.z = 2.0

		elif key_input == 'd':
			msg.angular.z = -2.0

		elif key_input == 's':
			msg.linear.x = -2.0
			msg.angular.z = 0.0

		elif key_input == 'q':
			print("Node Stop")
			break


		turtlesim_publisher.publisher.publish(msg)
		
	turtlesim_publisher.destroy_node()
	rp.shutdown()	
	
	
if __name__ == '__main__':
	main()
