import json
import numpy as np
import random

ranges = []
## for angle in range(360):
##    value = abs(np.sin(np.radians(angle))) * 3.0
##   ranges.append(round(value, 2))
range_min = 0.12
range_max = 3.5

ranges = [round(random.uniform(range_min, range_max), 2) for _ in range(360)]

with open("lds02_mock.json") as f:
    data = json.load(f)

data["ranges"] = ranges
data["intensities"] = [100] * 360  

front = np.r_[ranges[350:360], ranges[0:10]]
left  = ranges[80:100]
right = ranges[260:280]

front_dist = np.mean(front)
left_dist  = np.mean(left)
right_dist = np.mean(right)

safe_dist = 0.5  

if front_dist < safe_dist:
    action = "turn_left" if left_dist > right_dist else "turn_right"
else:
    action = "go_forward"

print("front:", round(front_dist, 2))
print("left :", round(left_dist, 2))
print("right:", round(right_dist, 2))
print("action:", action)


with open("lds02_mock.json", "w") as f:
    json.dump(data, f, indent=4)
