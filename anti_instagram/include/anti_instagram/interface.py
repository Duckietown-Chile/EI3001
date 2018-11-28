import duckietown_utils as dtu
from abc import ABCMeta, abstractmethod

__all__ = [
    'AntiInstagramInterface',
]


class AntiInstagramInterface(object):
    
    FAMILY = 'anti_instagram'
    
    __metaclass__ = ABCMeta
   
    
    @abstractmethod
    @dtu.contract(bgr='array[HxWx3](uint8)', returns='None')
    def calculateTransform(self, bgr):
        """ Computes the transform """

    @abstractmethod
    @dtu.contract(bgr='array[HxWx3](uint8)', returns='None')
    def applyTransform(self, bgr):
        """ Applies the transform """
    
    @abstractmethod
    def calculateHealth(self):
        """ Returns health. TODO: what is this exactly? """
        