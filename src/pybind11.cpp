#include "../include/vehicle_interfaces/timesync.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

PYBIND11_MODULE(utils, m)
{
    m.doc() = "vehicle_interfaces package built from pybind11";

    py::class_<Timer>(m, "Timer")
        .def(py::init<int, const std::function<void()>&>())
        .def("start", &Timer::start, "Start timer")
        .def("stop", &Timer::stop, "Stop timer")
        .def("destroy", &Timer::destroy, "Destroy timer");
    
    // py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node>>(m, "Node");
    // py::class_<TimeSyncNode, std::shared_ptr<TimeSyncNode>, rclcpp::Node>(m, "TimeSyncNode", py::multiple_inheritance())
    //     .def(py::init<const std::string&, std::string, int>())
    //     .def("syncTime", &TimeSyncNode::syncTime, "Sync time")
    //     .def("getTimestamp", &TimeSyncNode::getTimestamp, "Get timestamp");
}