import time
import roslibpy


client = roslibpy.Ros(host='localhost', port=9090)
client.run()

print("Connected:", client.is_connected)


def callback(msg):
    ranges = msg["ranges"]
    action(ranges)

def action(ranges):
    front = ranges[350:] + ranges[:11]   
    left  = ranges[75:106]               
    right = ranges[255:286]              

    front_min = min(front)
    left_min  = min(left)
    right_min = min(right)

    def near(v, target, tol=0.05):
        return abs(v - target) < tol

    if near(front_min, 3.5)  and near(left_min, 3.5) and near(right_min, 3.5): 
        print("장애물이 감지되지 않음..... 직진하십시오")
    elif near(front_min, 0.4) and near(right_min, 3.5) and near(left_min, 3.5):
        print("정면에서 장애물이 감지됨...  우회전 하십시오")
    elif near(front_min, 0.4) and near(right_min, 0.4):
        print("정면과 우측에서 장애물이 감지됨.....  후진 후 좌회전하십시오")
    elif near(front_min, 0.4) and near(left_min, 0.4):
        print("정면과 좌측에서 장애물이 감지됨.....  후진 후 우회전하십시오")
       

listener = roslibpy.Topic(
    client,
    '/set_data',
    'sensor_msgs/msg/LaserScan'
)

listener.subscribe(callback)

try:
    while client.is_connected:
        time.sleep(0.1)
except KeyboardInterrupt:
    client.terminate()