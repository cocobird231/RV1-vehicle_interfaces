from vehicle_interfaces.devinfo import DevInfoNode
from vehicle_interfaces.params import GenericParams
from vehicle_interfaces.qos import QoSUpdateNode
from vehicle_interfaces.safety import SafetyNode
from vehicle_interfaces.timesync import TimeSyncNode

from vehicle_interfaces.node_adaptor import NodeAdaptor

class VehicleServiceNode(DevInfoNode, QoSUpdateNode, SafetyNode, TimeSyncNode):
    def __init__(self, gParams : GenericParams):
        DevInfoNode.__init__(self, gParams.nodeName, gParams.devInfoService, gParams.devInterface, gParams.devMultiNode)
        QoSUpdateNode.__init__(self, gParams.nodeName, gParams.qosService, gParams.qosDirPath)
        SafetyNode.__init__(self, gParams.nodeName, gParams.safetyService)
        TimeSyncNode.__init__(self, gParams.nodeName, gParams.timesyncService, gParams.timesyncPeriod_ms, gParams.timesyncAccuracy_ms, gParams.timesyncWaitService)
        NodeAdaptor.reset()
