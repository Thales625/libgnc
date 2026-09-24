from typing import ClassVar
import numpy as np

from utils.quaternion import quaternion_from_euler

from core.earth import Earth

def target_position(t: float) -> np.ndarray:
    return np.array([
        np.sin(0.5*t),
        np.cos(0.5*t),
        0.# np.cos(1.*t)
    ])

class SimulationConfig:
    dt: ClassVar[float] = 10e-3
    ideal_navigation: ClassVar[bool] = True

class StaticSimulationConfig:
    total_time: ClassVar[float] = 1.2
    plot_dt:    ClassVar[float] = 5e-3

class LiveSimulationConfig:
    steps_per_frame: ClassVar[int] = 10

class AvionicsConfig:
    interval: ClassVar[float] = 3e-3
    control_sampling: ClassVar[int] = 1

class SensorsConfig:
    acc_interval:  ClassVar[float] = 1e-3
    acc_noise:     ClassVar[float] = 1e-3

    gyro_interval: ClassVar[float] = 1e-3
    gyro_noise:    ClassVar[float] = 0.01

    baro_interval: ClassVar[float] = 0.02
    baro_noise:    ClassVar[float] = 0.2

class RotorConfig:
    kt:    ClassVar[float] = 1e-6
    b:     ClassVar[float] = 2e-8
    rpm:   ClassVar[float] = 20e3
    tau:   ClassVar[float] = 50e-3
    noise: ClassVar[float] = 1e-3

    b_kt_ratio = b / kt

class DroneConfig:
    mass: ClassVar[float] = 0.2
    size: ClassVar[float] = 0.2

    moi = (mass * size**2) * np.array([0.5, 0.5,  1])
    weight = mass * Earth.g

class InitialState:
    s0: ClassVar[np.ndarray] = np.zeros(3)
    v0: ClassVar[np.ndarray] = np.zeros(3)
    q0: ClassVar[np.ndarray] = quaternion_from_euler(0., 0., 0., True)
    w0: ClassVar[np.ndarray] = np.array([0., 0., 0.])
