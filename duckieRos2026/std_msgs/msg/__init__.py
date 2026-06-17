"""Standard ROS Message Types"""

class String:
    def __init__(self, data=""):
        self.data = data

class Header:
    def __init__(self):
        self.seq = 0
        self.stamp = 0
        self.frame_id = ""

class Time:
    @staticmethod
    def now():
        import time
        return time.time()
