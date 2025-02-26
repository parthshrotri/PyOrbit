import numpy as np
import quaternion

class StarTracker():
    def __init__(self, bore_to_body):
        self.bore_to_body   = bore_to_body