#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "vehicle_interfaces/timer.h"

namespace py = pybind11;

PYBIND11_MODULE(cpplib, m)
{
    m.doc() = "vehicle_interfaces package built from pybind11";

    py::class_<vehicle_interfaces::LiteTimer>(m, "Timer")
        .def(py::init<int, const std::function<void()>&, bool>())
        .def("start", &vehicle_interfaces::LiteTimer::start, "Start timer")
        .def("stop", &vehicle_interfaces::LiteTimer::stop, "Stop timer")
        .def("setPeriod", &vehicle_interfaces::LiteTimer::setPeriod, "Set timer period", py::arg("period"))
        .def("destroy", &vehicle_interfaces::LiteTimer::destroy, "Destroy timer");

    m.def("make_shared_timer", &vehicle_interfaces::make_shared_timer, "Create shared timer", py::arg("period_ms"), py::arg("callback"), py::arg("MAX_SCHED_RR") = false);
    m.def("make_unique_timer", &vehicle_interfaces::make_unique_timer, "Create unique timer", py::arg("period_ms"), py::arg("callback"), py::arg("MAX_SCHED_RR") = false);
}