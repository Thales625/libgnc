import numpy as np

from .earth import Earth

class Sensor:
    C = np.array([])

    def get_output(self):
        output = self.C @ self.state
        return output + np.random.normal(0, self.noise, len(output))

    def __init__(self, state:np.ndarray, interval:float, noise:float):
        self.state = state
        self.interval = interval
        self.noise = noise

        self.output = self.get_output()
        self.data_ready = True

        self.last_ut = 0.

    def update(self, now:float) -> np.ndarray|None:
        # update reading
        if now - self.last_ut >= self.interval:
            self.last_ut = now
            self.output = self.get_output()
            self.data_ready = True

    def __call__(self) -> np.ndarray:
        self.data_ready = False
        return self.output


class Accelerometer(Sensor):
    C = np.array([
        #[ax, ay, az]
        [1./Earth.g, 0., 0.], # ax
        [0., 1./Earth.g, 0.], # ay
        [0., 0., 1./Earth.g], # az
    ])

    # override
    def get_output(self):
        output = self.C @ (self.state - Earth.gravity)
        return output + np.random.normal(0, self.noise, len(output))

    def __init__(self, accel_state:np.ndarray, interval=1e-3, noise=1e-3):
        super().__init__(accel_state, interval, noise)

class Gyroscope(Sensor):
    C = np.array([
       #[x,  y,  z,  vx, vy, vz, qw, qx, qy, qz, vr, vp, vy]
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.], # vr
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.], # vp
        [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.], # vy
    ])

    def __init__(self, state:np.ndarray, interval=1e-3, noise=0.01):
        super().__init__(state, interval, noise)

class Barometer(Sensor):
    C = np.array([
       #[x,  y,  z,  vx, vy, vz, qw, qx, qy, qz, vr, vp, vy]
        [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.] # z
    ])

    def __init__(self, state:np.ndarray, interval=0.02, noise=0.2):
        super().__init__(state, interval, noise)

class Magnetometer(Sensor):
    # override
    def get_output(self):
        # quat to yaw
        w, x, y, z = self.state[6:10]
        siny_cosy = 2.0 * (w*z + x*y)
        cosy_cosy = 1.0 - 2.0 * (y*y + z*z)
        output = np.array([np.arctan2(siny_cosy, cosy_cosy)])
        return output + np.random.normal(0, self.noise, len(output))

    def __init__(self, state:np.ndarray, interval=1/75, noise=0.017):
        super().__init__(state, interval, noise)

class GPS(Sensor):
    C = np.array([
       #[x,  y,  z,  vx, vy, vz, qw, qx, qy, qz, vr, vp, vy]
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.], # x
        [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.], # y
        [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.], # z
    ])

    def __init__(self, state:np.ndarray, interval=1.0, noise=2.5):
        super().__init__(state, interval, noise)

if __name__ == '__main__':
    from utils.quaternion import quaternion_from_euler

    S0 = np.array([0., 0., 0.])
    V0 = np.array([0., 0., 0.,])
    Q0 = quaternion_from_euler(0., 0., 0., True)
    W0 = np.array([0., 0., 0.])

    state = np.hstack([S0, V0, Q0, W0])
    accel_state = np.array([0., 0., 0.])

    acc = Accelerometer(accel_state)
    gyro = Gyroscope(state)
    mag = Magnetometer(state)
    gps = GPS(state)
    baro = Barometer(state)

    t = 0.
    for i in range(100):
        t += 0.1

        accel_state += np.array([1., 1., 1.])

        state[0:3] += np.array([1., 1., 1.])

        # update sensors
        for sensor in [acc, gyro, mag, gps, baro]:
            sensor.update(t)

        print(f"T: {t:.1f} ", end="")
        # print(f"State: {state} | Acc: {acc()}")
        # print(f"State: {state} | GPS: {gps()}")
        # print(f"State: {state} | Gyro: {gyro()}")
        # print(f"State: {state} | Baro: {baro()}")
        print(f"State: {state} | Mag: {mag()}")
