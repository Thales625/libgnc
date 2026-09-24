import numpy as np

class RK4:
    def __init__(self, state:np.ndarray, dSdt) -> None:
        self.S = state
        self.dSdt = dSdt

    def step(self, dt:float, u:np.ndarray) -> np.ndarray:
        k1 = self.dSdt(self.S, u)
        k2 = self.dSdt(self.S + 0.5*dt*k1, u)
        k3 = self.dSdt(self.S + 0.5*dt*k2, u)
        k4 = self.dSdt(self.S + dt*k3, u)
        self.S += (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
        return self.S
