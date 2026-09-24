import numpy as np

class Propulsion:
    def __init__(self, position_body:np.ndarray, direction_body:np.ndarray):
        self.position_body = position_body
        self.direction_body = direction_body

        self._throttle = 0.0

    def update(self, dt:float, t:float) -> None:
        raise NotImplementedError

    @property
    def throttle(self) -> float:
        return self._throttle
    @throttle.setter
    def throttle(self, value:float):
        if value > 1.:
            value = 1.
        elif value < 0.:
            value = 0.
        self._throttle = value

    @property
    def force(self) -> np.ndarray:
        raise NotImplementedError

    @property
    def torque(self) -> np.ndarray:
        raise NotImplementedError
