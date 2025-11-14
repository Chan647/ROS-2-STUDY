import time
import roslibpy
import threading

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

def callback(msg):
    x = msg['x']
    y = msg['y']
    theta = msg['theta']
    time_str = msg['timestr']
    
    print("x:", x)
    print("y:", y)
    print("theta:", theta)
    print("time:", time_str)

listener = roslibpy.Topic(client, '/turtle_time', 'turtle_com_package_msgs/TurtleMsg')
listener.subscribe(callback)
vel = roslibpy.Topic(client, 'turtle1/cmd_vel', 'geometry_msgs/Twist')
reset = roslibpy.Service(client, 'reset', 'std_srvs/Empty')

forward = roslibpy.Message({
    'linear': {'x': 2.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
})

backward = roslibpy.Message({
    'linear': {'x': -2.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
})

left = roslibpy.Message({
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 1.57}
})

right = roslibpy.Message({
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': -1.57}
})

def input_com():
    while client.is_connected:
        key_input = input("key : ")

        if key_input == 'w':
            vel.publish(forward)
        elif key_input == 'a':
            vel.publish(left)
        elif key_input == 's':
            vel.publish(backward)
        elif key_input == 'd':
            vel.publish(right)
        elif key_input == 'q':
            reset.call(roslibpy.ServiceRequest({}))

        time.sleep(0.2)

t = threading.Thread(target=input_com)
t.daemon = True
t.start()

try:
    while client.is_connected:
        time.sleep(0.1)
except KeyboardInterrupt:
    listener.unsubscribe()
    vel.unadvertise()
    client.terminate()


