import numpy as np

from core.earth import Earth

class DroneConfig:
    mass: ClassVar[float] = 0.2
    size: ClassVar[float] = 0.2

    moi = (mass * size**2) * np.array([0.5, 0.5,  1])
    weight = mass * Earth.g