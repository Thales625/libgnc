import numpy as np

from .config import DroneConfig

from config import SimulationConfig, RotorConfig, InitialState, SensorsConfig, AvionicsConfig
from config import target_position

from vehicles.quadcopter import Quadcopter

from core.sensors import Accelerometer, Gyroscope, Barometer
from core.rotor import Rotor

# SIL
from . import libavionics
from core.avionics import Avionics

avionics = Avionics(AvionicsConfig.interval, libavionics)

vehicle = Quadcopter(DroneConfig.mass, DroneConfig.moi, DroneConfig.size, avionics)
vehicle.position = InitialState.s0
vehicle.velocity = InitialState.v0
vehicle.rotation = InitialState.q0
vehicle.angular_velocity = InitialState.w0

rotor_fr = Rotor(
    np.array([DroneConfig.size, DroneConfig.size, 0.]),
    RotorConfig.kt,
    RotorConfig.b,
    RotorConfig.rpm,
    RotorConfig.noise,
    time_constant=RotorConfig.tau,
    clockwise=True,
)
rotor_fl = Rotor(
    np.array([DroneConfig.size, -DroneConfig.size, 0.]),
    RotorConfig.kt,
    RotorConfig.b,
    RotorConfig.rpm,
    RotorConfig.noise,
    time_constant=RotorConfig.tau,
    clockwise=False,
)

rotor_br = Rotor(
    np.array([-DroneConfig.size, DroneConfig.size, 0.]),
    RotorConfig.kt,
    RotorConfig.b,
    RotorConfig.rpm,
    RotorConfig.noise,
    time_constant=RotorConfig.tau,
    clockwise=False,
)
rotor_bl = Rotor(
    np.array([-DroneConfig.size, -DroneConfig.size, 0.]),
    RotorConfig.kt,
    RotorConfig.b,
    RotorConfig.rpm,
    RotorConfig.noise,
    time_constant=RotorConfig.tau,
    clockwise=True,
)

vehicle.add_propulsion(rotor_fr)
vehicle.add_propulsion(rotor_fl)

vehicle.add_propulsion(rotor_br)
vehicle.add_propulsion(rotor_bl)

accelerometer = vehicle.add_sensor("accelerometer", Accelerometer(vehicle.accel_body_state, interval=SensorsConfig.acc_interval, noise=SensorsConfig.acc_noise))
gyroscope = vehicle.add_sensor("gyroscope", Gyroscope(vehicle.state, interval=SensorsConfig.gyro_interval, noise=SensorsConfig.gyro_noise))
barometer = vehicle.add_sensor("barometer", Barometer(vehicle.state, interval=SensorsConfig.baro_interval, noise=SensorsConfig.baro_noise))

vehicle.avionics.core.data.armed = True

def sim_loop(t:float) -> None:
    # update simulation
    vehicle.update(SimulationConfig.dt, t)

    # send target state to avionics
    tgt_pos = target_position(t)
    tgt_state = np.concatenate((tgt_pos, np.zeros(9)))
    vehicle.avionics.core.set_target_state(tgt_state)

    # update sensors
    acc_val  = accelerometer()
    gyro_val = gyroscope()
    baro_val = barometer()

    # force navigation - DEBUG
    vehicle.avionics.core.set_state(vehicle.state)

    # update avionics data
    vehicle.avionics.core.data.ut = int(t*1000) # ms
    vehicle.avionics.core.data.accel = acc_val
    vehicle.avionics.core.data.gyro = gyro_val
    vehicle.avionics.core.data.altitude = float(baro_val[0])

    # update avionics
    vehicle.avionics.update(t)

    # apply control
    # for i, cmd in enumerate(vehicle.avionics.core.data.actuator_cmd):
        # if i >= 3: break
    for i in range(len(vehicle.propulsions)):
        cmd = vehicle.avionics.core.data.actuator_cmd[i]
        vehicle.propulsions[i].throttle = cmd