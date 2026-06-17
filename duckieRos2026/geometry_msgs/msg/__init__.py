"""Geometry Message Types"""

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()

class Pose:
    def __init__(self):
        self.position = Vector3()
        self.orientation = Quaternion()
