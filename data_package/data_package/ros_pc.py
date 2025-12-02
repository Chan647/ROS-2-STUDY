import rclpy as rp
import numpy as np
import random
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5

def Make_obj(ranges, center_deg, width_deg, distance=0.4):
    half_width = width_deg // 2
    for offset in range(-half_width, half_width + 1):
        idx = (center_deg + offset) % NUM_POINTS
        ranges[idx] = distance
    random_idx = random.randrange(0, 359)
    ranges[random_idx] = np.inf

def Not(ranges):
    for i in range(NUM_POINTS):
        ranges[i] = RANGE_MAX
        
def Front(ranges):
    Make_obj(ranges, center_deg=0, width_deg=40)


def Left(ranges):
    Make_obj(ranges, center_deg=90, width_deg=30)


def Right(ranges):
    Make_obj(ranges, center_deg=270, width_deg=30)


def Front_Right(ranges):
    Front(ranges)
    Right(ranges)


def Front_Left(ranges):
    Front(ranges)
    Left(ranges)


PATTERNS = [
    Not,
    Front,
    Left,
    Right,
    Front_Right,
    Front_Left
]
    

class Makedata(Node):

    def __init__(self):
        super().__init__('make_data')
        self.pub = self.create_publisher(LaserScan, 'set_data',10)
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        ranges = [RANGE_MAX] * NUM_POINTS
        intensities = [float(100) for _ in range(NUM_POINTS)]
        
        pattern = random.choice(PATTERNS)
        pattern_name = pattern.__name__
        pattern(ranges)

        msg = LaserScan()
        msg.angle_min = math.radians(ANGLE_MIN_DEG)
        msg.angle_max = math.radians(ANGLE_MAX_DEG)
        msg.angle_increment = math.radians(ANGLE_INCREMENT_DEG)
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        msg.intensities = intensities
        self.pub.publish(msg)
        print(msg)


def main(args=None):
    rp.init(args=args)

    make_data = Makedata()
    rp.spin(make_data)

    make_data.destory_node()
    rp.shutdown()

if __name__ == '__main__':
    main()