import rclpy
from rclpy.node import Node

class NodeAdaptor(Node):
    __constructorChk = False
    @classmethod
    def check(cls):
        if (not cls.__constructorChk):
            cls.__constructorChk = True
            return True
        return False
    
    @classmethod
    def reset(cls):
        cls.__constructorChk = False
    
    def __init__(self, nodeName : str):
        if (NodeAdaptor.check()):
            super().__init__(nodeName)