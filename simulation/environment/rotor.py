import numpy as np

class Rotor:
    def __init__(self, pos_body_ref, kt, b, max_rpm, noise=1e-8, time_constant=0.05, clockwise=True):
        self.pos = pos_body_ref
        self.clockwise = clockwise

        # params
        self.kt = kt
        self.b = b
        self.tau = time_constant
        self.noise = noise

        self._control = 0.
        self.thrust = 0.
        self.torque = 0.

        self.ang_vel = 0.
        self.max_ang_vel = max_rpm * 0.10472 # rad/s

    def update(self, dt):
        target_vel = self._control * self.max_ang_vel

        alpha = np.exp(-dt / self.tau)
        self.ang_vel = alpha * self.ang_vel + (1 - alpha) * target_vel

        self.thrust = self.kt * self.ang_vel*abs(self.ang_vel)
        self.thrust += np.random.normal(0, self.noise)
        if self.thrust < 0: self.thrust = 0.

        self.torque = (1 if self.clockwise else -1) * self.b * self.ang_vel*abs(self.ang_vel)

    @property
    def control(self) -> float:
        return self._control
    @control.setter
    def control(self, value:float):
        if value > 1.:
            value = 1.
        elif value < 0.:
            value = 0.
        self._control = value

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    from simulation.config import DroneConfig, RotorConfig

    rotor = Rotor(
        np.array([DroneConfig.size, DroneConfig.size, 0.]),
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
        rpm_arr.append(rotor.ang_vel / 0.10472)

    plt.plot(time_arr, thrust_arr, label="Thrust (N)")
    plt.plot(time_arr, np.array(control_arr)*max(thrust_arr), label="Control")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(rpm_arr, thrust_arr, label="Thrust (N)")
    plt.legend()
    plt.grid()
    plt.show()
