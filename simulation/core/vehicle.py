import numpy as np
from scipy.signal import cont2discrete

from utils import quaternion_to_euler, quaternion_from_euler

from .sensors import Sensor
from .solver import RK4

class Vehicle:
    def __init__(self, mass:float, moi:np.ndarray):
        self.mass = mass
        self.moi = moi

        self.state = np.array([
            0., 0., 0., # position | Vec3
            0., 0., 0., # velocity | Vec3

            1., 0., 0., 0., # rotation | Quat
            0., 0., 0.,     # ang vel | Vec3
        ])

        self.propulsions = []
        self.sensors = []

        # solver
        self.solver = RK4(self.state, self.dynamics)

    def u(self, force_body:np.ndarray, torque_body:np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def dynamics(self, x:np.ndarray, u:np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def update(self, dt:float, t:float) -> None:
        force_body = np.zeros(3)
        torque_body = np.zeros(3)

        for propulsion in self.propulsions:
            propulsion.update(dt, t)

            force_body += propulsion.force
            torque_body += propulsion.torque

        # step solver
        self.state = self.solver.step(dt, self.u(force_body, torque_body))

        # normalize rotation quat
        norm = np.linalg.norm(self.rotation)
        if norm > 1e-6:
            self.rotation /= norm
        else:
            self.rotation = np.array([1.0, 0.0, 0.0, 0.0])

        # update sensors
        for sensor in self.sensors:
            sensor.update(t)

    def add_propulsion(self, propulsion:Propulsion) -> Propulsion:
        self.propulsions.append(propulsion)
        return propulsion

    def add_sensor(self, sensor:Sensor) -> Sensor:
        self.sensors.append(sensor)
        return sensor

    def linearize(self, x_eq:np.ndarray, u_eq:np.ndarray, dt:float, eps=1e-6) -> list[np.ndarray]:
        def dynamics(x:self.state, u:self.state): # dynamics transform (model_state -> state space)
            return self.state_to_model_state(self.dynamics(self.model_state_to_state(x), u))

        # A matrix
        n = x_eq.shape[0]
        A = np.zeros((n, n))

        for i in range(n):
            dx = np.zeros(n)
            dx[i] = eps

            f_plus  = dynamics(x_eq + dx, u_eq)
            f_minus = dynamics(x_eq - dx, u_eq)

            A[:, i] = (f_plus - f_minus) / (2 * eps)

        # B matrix
        m = u_eq.shape[0]
        B = np.zeros((n, m))

        for i in range(m):
            du = np.zeros(m)
            du[i] = eps

            f_plus  = dynamics(x_eq, u_eq + du)
            f_minus = dynamics(x_eq, u_eq - du)

            B[:, i] = (f_plus - f_minus) / (2 * eps)

        # discretize
        C = np.eye(12)
        D = np.zeros((12, 4))
        Ad, Bd, Cd, Dd, _ = cont2discrete( # type: ignore
            (A, B, C, D),
            dt,
            method='zoh'
        )

        return (Ad, Bd)

    @staticmethod
    def state_to_model_state(state: np.ndarray) -> np.ndarray:
        x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz = state

        roll, pitch, yaw = quaternion_to_euler(np.array([qw, qx, qy, qz]))

        return np.array([
            x, y, z,
            vx, vy, vz,
            roll, pitch, yaw,
            wx, wy, wz
        ])

    @staticmethod
    def model_state_to_state(model_state: np.ndarray) -> np.ndarray:
        x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz = model_state

        quat = quaternion_from_euler(roll, pitch, yaw)

        return np.concatenate([
            [x, y, z],
            [vx, vy, vz],
            quat,
            [wx, wy, wz]
        ])

    @property
    def model_state(self) -> np.ndarray:
        return self.state_to_model_state(self.state)

    @property
    def position(self) -> np.ndarray:
        return self.state[0:3]
    @position.setter
    def position(self, value:np.ndarray):
        self.state[0:3] = value

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6]
    @velocity.setter
    def velocity(self, value:np.ndarray):
        self.state[3:6] = value

    @property
    def rotation(self) -> np.ndarray:
        return self.state[6:10]
    @rotation.setter
    def rotation(self, value:np.ndarray):
        self.state[6:10] = value

    @property
    def euler_angles(self) -> np.ndarray:
        return quaternion_to_euler(self.rotation)
    @euler_angles.setter
    def euler_angles(self, value:np.ndarray):
        self.rotation = quaternion_from_euler(*value)

    @property
    def angular_velocity(self) -> np.ndarray:
        return self.state[10:13]
    @angular_velocity.setter
    def angular_velocity(self, value:np.ndarray):
        self.state[10:13] = value