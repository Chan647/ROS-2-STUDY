import json
import math
import os
import random

ANGLE_MIN_DEG = 0
ANGLE_MAX_DEG = 359
ANGLE_INCREMENT_DEG = 1
NUM_POINTS = 360
RANGE_MIN = 0.12
RANGE_MAX = 3.5
WITH_DEG = 40
CENTER_DEG = 0

ranges = [round(random.uniform(RANGE_MIN, RANGE_MAX), 2) for _ in range(360)]
#half_width = WITH_DEG // 2
#for offset in range(-half_width, half_width + 1):
    #idx = (CENTER_DEG + offset) % NUM_POINTS
    #ranges[idx] = 0.4

print(ranges)