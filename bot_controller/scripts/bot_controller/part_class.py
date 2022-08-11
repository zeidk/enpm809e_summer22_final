#!/usr/bin/env python3

from geometry_msgs.msg import Pose

class Part(object):
    """
    Class used to store part information retrieved from a logical camera
    """
    
    def __init__(self, model, frame=None, pose=None):
        self._model = model
        self._pose = pose
        self._frame = frame
        
    @property
    def model(self):
        return self._model
    
        
    @property
    def pose(self):
        return self._pose
        
    @property
    def frame(self):
        return self._frame

        
