import math
import numpy
from math import atan2, pi, ceil, floor, cos, sin

INFINIT = "__FLT_MAX__"

FRONT_LIMIT = 320
SIDE_LIMIT = 240
ROBOT_FRONT = 170
NUM_ANGLE = 360

end_angle = ceil(atan2(SIDE_LIMIT,  ROBOT_FRONT) / pi * 180)
front_angle = floor(atan2(SIDE_LIMIT,  FRONT_LIMIT) / pi * 180)
print("Front_angle = " + str(front_angle))
print("End_angle = " + str(end_angle))

limit = [INFINIT for i in range(NUM_ANGLE)]

for i in range(NUM_ANGLE):
    if i == 0:
        limit[i] = str(FRONT_LIMIT)
    if (0 < i <= front_angle) or (NUM_ANGLE - front_angle <= i < NUM_ANGLE):
        limit[i] = str(round(abs(FRONT_LIMIT / cos(i / 180 * pi)), 2))
    if (front_angle < i <= end_angle) or (NUM_ANGLE-end_angle <= i < NUM_ANGLE - front_angle):
        limit[i] = str(round(abs(SIDE_LIMIT / sin(i / 180 * pi)), 2))

limit_short = []
for i in range(NUM_ANGLE):
    if i == 0:
        limit_short.append(str(FRONT_LIMIT))
    if (0 < i <= front_angle) or (NUM_ANGLE - front_angle <= i < NUM_ANGLE):
        limit_short.append(str(round(abs(FRONT_LIMIT / cos(i / 180 * pi)), 2)))
    if (front_angle < i <= end_angle) or (NUM_ANGLE-end_angle <= i < NUM_ANGLE - front_angle):
        limit_short.append(str(round(abs(SIDE_LIMIT / sin(i / 180 * pi)), 2)))

object_range = len(limit_short)
limit_short = "{" + ", ".join(limit_short) + "};"
limit = "{" + ", ".join(limit) + "};"

print("limit:")
print(limit)
print("limit_short:")
print(limit_short)

print("OBJECT_RANGE =")
print(object_range)


