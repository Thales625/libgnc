import numpy as np


from core.propulsion import Propulsion

class Rotor(Propulsion):
    def __init__(self, position_body:np.ndarray, kt:float, b:float, max_rpm:float, noise=1e-8, time_constant=0.05, clockwise=True):
        super().__init__(position_body, np.array([0., 0., 1.]))

        # params
        self.clockwise = clockwise
        self.kt = kt
        self.b = b
        self.tau = time_constant
        self.noise = noise
        self.max_rotation = max_rpm * 0.10472 # rad/s

        self._rotation = 0.
        self._thrust = 0.
        self._force = np.zeros(3)
        self._torque = np.zeros(3)

    # override
    def update(self, dt:float, t:float) -> None:
        target_rotation = self._throttle * self.max_rotation

        alpha = np.exp(-dt / self.tau)
        self._rotation = alpha * self._rotation + (1 - alpha) * target_rotation

        self._thrust = self.kt * self._rotation*abs(self._rotation)
        self._thrust += np.random.normal(0, self.noise)
        if self._thrust < 0: self._thrust = 0.
        self._force = self._thrust * self.direction_body

        self._torque = np.cross(self.position_body, self._force)
        self._torque += ((1 if self.clockwise else -1) * self.b * self._rotation*abs(self._rotation))*self.direction_body # coriolis torque

    # override
    @property
    def force(self) -> np.ndarray:
        return self._force

    # override
    @property
    def torque(self) -> np.ndarray:
        return self._torque

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    from simulation.config import RotorConfig

    rotor = Rotor(
        np.array([0., 0., 0.]),
        RotorConfig.kt,
        RotorConfig.b,
        RotorConfig.rpm,
        RotorConfig.noise,
        time_constant=RotorConfig.tau,
        clockwise=True,
    )

    dt = 0.01

    time_arr = []
    thrust_arr = []
    control_arr = []
    rpm_arr = []

    for t in np.arange(0., 10., dt):
        control = 0.
        if t > 5 and t < 20:
            control = 1.

        rotor.control = control

        rotor.update(dt)

        time_arr.append(t)
        thrust_arr.append(rotor.thrust)
        control_arr.append(rotor.control)
        rpm_arr.append(rotor.rotation / 0.10472)

    plt.plot(time_arr, thrust_arr, label="Thrust (N)")
    plt.plot(time_arr, np.array(control_arr)*max(thrust_arr), label="Control")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(rpm_arr, thrust_arr, label="Thrust (N)")
    plt.legend()
    plt.grid()
    plt.show()
