import rclpy
from rclpy.node import Node

class GenericParams(Node):
    def __init__(self, nodeName : str):
        super().__init__(nodeName)

        self.nodeName = 'node'
        self.id = 0
        self.qosService = 'qos_0'
        self.safetyService = 'safety_0'
        self.timesyncService = 'timesync_0'
        self.timesyncInterval_ms = 1000.0
        self.timesyncAccuracy_ms = 2.0

        self.declare_parameter('nodeName', self.nodeName)
        self.declare_parameter('id', self.id)
        self.declare_parameter('qosService', self.qosService)
        self.declare_parameter('safetyService', self.safetyService)
        self.declare_parameter('timesyncService', self.timesyncService)
        self.declare_parameter('timesyncInterval_ms', self.timesyncInterval_ms)
        self.declare_parameter('timesyncAccuracy_ms', self.timesyncAccuracy_ms)
        self._getParam()
    
    def _getParam(self):
        self.nodeName = rclpy.parameter.parameter_value_to_python(self.get_parameter('nodeName').get_parameter_value())
        self.id = rclpy.parameter.parameter_value_to_python(self.get_parameter('id').get_parameter_value())
        self.qosService = rclpy.parameter.parameter_value_to_python(self.get_parameter('qosService').get_parameter_value())
        self.safetyService = rclpy.parameter.parameter_value_to_python(self.get_parameter('safetyService').get_parameter_value())
        self.timesyncService = rclpy.parameter.parameter_value_to_python(self.get_parameter('timesyncService').get_parameter_value())
        self.timesyncInterval_ms = rclpy.parameter.parameter_value_to_python(self.get_parameter('timesyncInterval_ms').get_parameter_value())
        self.timesyncAccuracy_ms = rclpy.parameter.parameter_value_to_python(self.get_parameter('timesyncAccuracy_ms').get_parameter_value())
        self.nodeName += '_' + str(self.id) + '_node'