"""Sensor Message Types"""

from std_msgs.msg import Header

class Image:
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = False
        self.step = 0
        self.data = b""

class CompressedImage:
    def __init__(self):
        self.header = Header()
        self.format = ""
        self.data = b""
