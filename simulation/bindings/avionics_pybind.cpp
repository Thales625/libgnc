#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "libgnc_pybind_casters.hpp"

#include <avionics.hpp>

namespace py = pybind11;

PYBIND11_MODULE(libavionics, m)
{
    // FlightPhase enum
    py::enum_<FlightPhase>(m, "FlightPhase")
        .value("PHASE_WAITING", PHASE_WAITING)
        .value("PHASE_FLIGHT", PHASE_FLIGHT)
        .value("PHASE_AUTO_LAND", PHASE_AUTO_LAND);

    // Avionics class
    py::class_<Avionics>(m, "Avionics")
        .def(py::init<>())

        .def("update", &Avionics::update)
        .def("estimated_state", &Avionics::estimated_state)
        .def("estimated_position", &Avionics::estimated_position)
        .def("estimated_velocity", &Avionics::estimated_velocity)
        .def("set_state", &Avionics::set_state)
        .def("set_target_state", &Avionics::set_target_state)

        .def_readwrite("ut", &Avionics::ut)
        .def_readwrite("phase", &Avionics::phase)
        .def_readwrite("accel", &Avionics::accel)
        .def_readwrite("ang_vel", &Avionics::ang_vel)
        .def_readwrite("altitude_baro", &Avionics::altitude_baro)
        .def_readwrite("armed", &Avionics::armed)
        .def_readwrite("rotor_cmd", &Avionics::rotor_cmd);
}