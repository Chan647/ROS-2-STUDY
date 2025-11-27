import time
import roslibpy


client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print("Connected:", client.is_connected)

vel = roslibpy.Topic(client, 'turtle1/cmd_vel', 'geometry_msgs/Twist')
pen = roslibpy.Service(client, '/turtle1/set_pen', 'turtlesim/SetPen')
req = roslibpy.ServiceRequest({'off':1})

pen.call(req)
print("Pen Off")

forward = roslibpy.Message({'linear': {'x': 2.0}})
backward = roslibpy.Message({'linear': {'x': -2.0}})
turn_left = roslibpy.Message({'linear': {'x': 2.0},'angular': {'z': 1.4}})
turn_right = roslibpy.Message({'linear': {'x': 2.0,},'angular': {'z': -1.4}})

state = "normal"

def callback(msg):
    ranges = msg["ranges"]
    action(ranges)

def action(ranges):
    global state 

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
        state = "forward"

    elif near(front_min, 0.4) and near(right_min, 3.5) and near(left_min, 3.5):
        print("정면에서 장애물이 감지됨...  우회전 하십시오")
        state = "turn_right"
        
    elif near(front_min, 0.4) and near(right_min, 0.4):
        print("정면과 우측에서 장애물이 감지됨.....  후진 후 좌회전하십시오")
        state = "backward_turn_left"
        
    elif near(front_min, 0.4) and near(left_min, 0.4):
        print("정면과 좌측에서 장애물이 감지됨.....  후진 후 우회전하십시오")
        state = "backward_turn_right"
        


listener = roslibpy.Topic(
    client,
    '/set_data',
    'sensor_msgs/msg/LaserScan'
)

listener.subscribe(callback)

try:
    while client.is_connected:
        if state == "forward":
            time.sleep(0.3)
            vel.publish(forward)
            state = "normal"
        
        elif state == "turn_right":
            time.sleep(0.3)
            vel.publish(turn_right)
            state = "normal"
        
        elif state == "backward_turn_left":
            time.sleep(0.3)
            vel.publish(backward)
            time.sleep(1.0)
            vel.publish(turn_left)
            state = "normal"

        elif state == "backward_turn_right":
            time.sleep(0.3)
            vel.publish(backward)
            time.sleep(1.0)
            vel.publish(turn_right)
            state = "normal"
        
except KeyboardInterrupt:
    client.terminate()