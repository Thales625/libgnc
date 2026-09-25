cmake_minimum_required(VERSION 3.20)

project(gnc_sil)

# ---- Scenario selection ----
set(SCENARIO quadcopter CACHE STRING "Simulation scenario")

message(STATUS "Building scenario: ${SCENARIO}")

# ---- Python / pybind11 ----
find_package(Python3 COMPONENTS Interpreter Development)

set(PYBIND11_FINDPYTHON ON)

find_package(pybind11 REQUIRED)

# ---- Python module ----
pybind11_add_module(libavionics bindings/avionics_pybind.cpp)

# Output
set_target_properties(
    libavionics
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
        ${PROJECT_SOURCE_DIR}/scenarios/${SCENARIO}
)

# ---- Includes ----
target_include_directories(
    libavionics PRIVATE

    # LibGNC
    ${PROJECT_SOURCE_DIR}/../include

    # Simulation common headers
    ${PROJECT_SOURCE_DIR}/include

    # Selected scenario
    ${PROJECT_SOURCE_DIR}/scenarios/${SCENARIO}
)

# ---- Compile definitions ----
target_compile_definitions(
    libavionics PRIVATE
    SIMULATION_BUILD
    SCENARIO_NAME="${SCENARIO}"
)