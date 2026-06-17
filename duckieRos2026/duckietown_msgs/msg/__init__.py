"""Duckietown Message Types"""

from std_msgs.msg import Header, String

class WheelsCmd:
    def __init__(self):
        self.header = Header()
        self.vel_left = 0.0
        self.vel_right = 0.0

class ChangePattern:
    def __init__(self):
        self.pattern_name = ""
        self.rgb_vals = [0, 0, 0]

class BoolStamped:
    def __init__(self):
        self.header = Header()
        self.data = False
