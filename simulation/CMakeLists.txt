cmake_minimum_required(VERSION 3.20)

project(gnc_sil)

find_package(
    Python
    COMPONENTS Interpreter Development
    REQUIRED
)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

pybind11_add_module(
    libavionics
    bindings/avionics_pybind.cpp
)

set_target_properties(
    libavionics
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
        ${PROJECT_SOURCE_DIR}/bindings
)

target_include_directories(
    libavionics PRIVATE
    ${PROJECT_SOURCE_DIR}/../include
)

target_include_directories(
    libavionics PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_definitions(
    libavionics PRIVATE
    SIMULATION_BUILD
)