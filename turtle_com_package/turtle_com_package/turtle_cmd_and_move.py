import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose
from turtle_com_package_msgs.msg import TurtleMsg
from datetime import datetime


class Turtleinformation(Node):

    def __init__(self):
        super().__init__('turtle_time_publisher')
        self.subscription = self.create_subscription(Pose,'/turtle1/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(TurtleMsg,'/turtle_time',10)
        self.timer_period = 2.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def pose_callback(self, msg):
        time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.msgs = TurtleMsg()
        self.msgs.x = msg.x
        self.msgs.y = msg.y
        self.msgs.theta = msg.theta
        self.msgs.timestr = time

        
    def timer_callback(self):
        self.publisher.publish(self.msgs)


def main(args=None):
    rp.init(args=args)
    node = Turtleinformation()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()