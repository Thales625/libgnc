#pragma once

#include "libgnc/math/vector.hpp"

enum FlightPhase {
    PHASE_WAITING,
    PHASE_FLIGHT,
    PHASE_AUTO_LAND
};

struct FlightData {
    // sensors
    gnc::Vec3f accel;
    gnc::Vec3f gyro;
    gnc::Vec3f mag;

    float altitude;
    float pressure;

    gnc::Vec3f gps_position;
    gnc::Vec3f gps_velocity;

    // actuators
    gnc::Vec<float, 8> actuator_cmd;

    // state machine
    FlightPhase phase;

    // system
    uint32_t ut;
    bool armed;
};

class FlightSoftware {
public:
    // ctor
    constexpr FlightSoftware() = default;

    // dtor
    virtual ~FlightSoftware() = default;

    // data
    FlightData data;

    // methods
    virtual void setup() = 0;
    virtual void update() = 0;
};