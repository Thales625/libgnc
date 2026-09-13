from environment.sensors import Accelerometer, Gyroscope, Barometer
from environment.drone import Drone
from environment.rotor import Rotor
from config import SimulationConfig, DroneConfig, AvionicsConfig, RotorConfig, InitialState, SensorsConfig
from config import target_position

from bindings.avionics_wrapper import AvionicsSim

import numpy as np

class EmbeddedBackend():
    def __init__(self):
        self.drone = Drone(DroneConfig.mass, DroneConfig.moi, DroneConfig.size)
        self.drone.position = InitialState.s0
        self.drone.velocity = InitialState.v0
        self.drone.rotation = InitialState.q0
        self.drone.angular_velocity = InitialState.w0

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

        self.drone.add_rotor(rotor_fr)
        self.drone.add_rotor(rotor_fl)

        self.drone.add_rotor(rotor_br)
        self.drone.add_rotor(rotor_bl)

        # control
        self.target_state = np.zeros(12)
        self.optimal_control = np.zeros(4)

        # sensors
        self.accelerometer = self.drone.add_sensor(Accelerometer(self.drone.accel_body_state, interval=SensorsConfig.acc_interval, noise=SensorsConfig.acc_noise))
        self.gyro = self.drone.add_sensor(Gyroscope(self.drone.state, interval=SensorsConfig.gyro_interval, noise=SensorsConfig.gyro_noise))
        self.baro = self.drone.add_sensor(Barometer(self.drone.state, interval=SensorsConfig.baro_interval, noise=SensorsConfig.baro_noise))

        # avionics
        self.avionics = AvionicsSim(AvionicsConfig.interval)

        # init
        self.avionics.core.armed = 1

    def sim_loop(self, t):
        # update simulation
        self.drone.update(SimulationConfig.dt, t)

        # update target
        self.target_state[:3] = target_position(t)

        # send target to avionics
        self.avionics.core.set_target_state(self.target_state)

        # update sensors
        acc_val  = self.accelerometer()
        gyro_val = self.gyro()
        baro_val = self.baro()

        # force navigation - DEBUG
        self.avionics.core.set_state(self.drone.state)

        # update avionics
        self.avionics.core.ut = int(t*1000) # ms
        self.avionics.core.accel = acc_val
        self.avionics.core.ang_vel = gyro_val
        self.avionics.core.altitude_baro = float(baro_val[0])

        self.avionics.update(t)

        # capture control input
        '''
        self.optimal_control[0] = self.avionics.controller.thrust
        self.optimal_control[1] = self.avionics.controller.roll_torque
        self.optimal_control[2] = self.avionics.controller.pitch_torque
        self.optimal_control[3] = self.avionics.controller.yaw_torque
        '''

        # apply control
        for i, cmd in enumerate(self.avionics.core.rotor_cmd):
            self.drone.rotors[i].control = cmd

    @property
    def estimated_state(self):
        return self.avionics.core.estimated_state()
