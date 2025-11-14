import time
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

print("Connected:", client.is_connected)

def callback(msg):
    print("x:", msg['x'])
    print("y:", msg['y'])
    print("theta:", msg['theta'])
    print("time:", msg['timestr'])

listener = roslibpy.Topic(
    client,
    '/turtle_time',
    'turtle_com_package_msgs/TurtleMsg'
)

listener.subscribe(callback)

try:
    while client.is_connected:
        time.sleep(0.1)
except KeyboardInterrupt:
    client.terminate()
