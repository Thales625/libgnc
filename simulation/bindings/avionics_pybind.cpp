#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "libgnc_pybind_casters.hpp"

#include "avionics.hpp"

namespace py = pybind11;

PYBIND11_MODULE(libavionics, m)
{
    py::class_<FlightData>(m, "FlightData")
        .def(py::init<>())

        .def_readwrite("accel", &FlightData::accel)
        .def_readwrite("gyro", &FlightData::gyro)
        .def_readwrite("mag", &FlightData::mag)

        .def_readwrite("altitude", &FlightData::altitude)
        .def_readwrite("pressure", &FlightData::pressure)

        .def_readwrite("gps_position", &FlightData::gps_position)
        .def_readwrite("gps_velocity", &FlightData::gps_velocity)

        .def_readwrite("actuator_cmd", &FlightData::actuator_cmd)

        .def_readwrite("phase", &FlightData::phase)

        .def_readwrite("ut", &FlightData::ut)
        .def_readwrite("armed", &FlightData::armed);

    // FlightPhase enum
    py::enum_<FlightPhase>(m, "FlightPhase")
        .value("PHASE_WAITING", PHASE_WAITING)
        .value("PHASE_FLIGHT", PHASE_FLIGHT)
        .value("PHASE_AUTO_LAND", PHASE_AUTO_LAND);

    // Avionics class
    py::class_<Avionics>(m, "Avionics")
        .def(py::init<>())

        .def("setup", &Avionics::setup)
        .def("update", &Avionics::update)

        .def("estimated_state", &Avionics::estimated_state)
        .def("estimated_position", &Avionics::estimated_position)
        .def("estimated_velocity", &Avionics::estimated_velocity)

        .def("get_target_state", &Avionics::get_target_state)

        .def("set_state", &Avionics::set_state)
        .def("set_target_state", &Avionics::set_target_state)

        .def_readwrite("data", &Avionics::data);
}