#pragma once
#include <rclcpp/rclcpp.hpp>
#include "devinfo.h"
#include "params.h"
#include "qos.h"
#include "safety.h"
#include "timesync.h"

namespace vehicle_interfaces
{

class VehicleServiceNode : public DevInfoNode, public QoSUpdateNode, public SafetyNode, public TimeSyncNode
{
public:
    VehicleServiceNode(const std::shared_ptr<GenericParams>& gParams) : 
        DevInfoNode(gParams->nodeName, gParams->devInfoService, gParams->devInterface, gParams->devMultiNode), 
        QoSUpdateNode(gParams->nodeName, gParams->qosService, gParams->qosDirPath), 
        SafetyNode(gParams->nodeName, gParams->safetyService), 
        TimeSyncNode(gParams->nodeName, gParams->timesyncService, gParams->timesyncPeriod_ms, gParams->timesyncAccuracy_ms, gParams->timesyncWaitService), 
        rclcpp::Node(gParams->nodeName) {}
};

}
