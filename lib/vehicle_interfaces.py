from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.qos import QoSUpdateNode
from vehicle_interfaces.safety import SafetyNode
from vehicle_interfaces.timesync import TimeSyncNode
from vehicle_interfaces.node_adaptor import NodeAdaptor

class VehicleServiceNode(QoSUpdateNode, SafetyNode, TimeSyncNode):
    def __init__(self, params : GenericParams):
        QoSUpdateNode.__init__(self, params.nodeName, params.qosService, params.qosDirPath)
        SafetyNode.__init__(self, params.nodeName, params.safetyService)
        TimeSyncNode.__init__(self, params.nodeName, params.timesyncService, params.timesyncPeriod_ms, params.timesyncAccuracy_ms)
        NodeAdaptor.reset()
