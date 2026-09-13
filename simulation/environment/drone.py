import numpy as np
from scipy.signal import cont2discrete

from utils import quaternion_multiply, quaternion_conjugate, quaternion_to_euler, rotate_vector
from utils.quaternion import quaternion_from_euler
from .rotor import Rotor
from .sensors import Sensor
from .earth import Earth
from .solver import RK4

from config import RotorConfig, AvionicsConfig

class Drone:
    def __init__(self, mass:float, moi:np.ndarray, size:float):
        self.state = np.array([
            0., 0., 0., # position | Vec3
            0., 0., 0., # velocity | Vec3

            1., 0., 0., 0., # rotation | Quat
            0., 0., 0.,     # ang vel | Vec3
        ])

        self.accel_body_state = np.array([0., 0., 0.])

        self.mass = mass
        self.moi = moi
        self.size = size
        self.rotors = []
        self.sensors = []

        # drag
        self.k_linear_drag = 0.4 * (self.size**2)
        self.k_rot_drag = 0.01

        # aux variables
        self.available_thrust = 0.
        self.available_torque = np.array([0., 0., 0.]) # r, p, y

        # solver
        self.solver = RK4(self.state, self.dynamics)

    def dynamics(self, x:np.ndarray, u:np.ndarray):
        pos = x[0:3]
        vel = x[3:6]
        rot = x[6:10]
        ang_vel = x[10:13]

        thrust, roll_torque, pitch_torque, yaw_torque = u

        thrust_body = np.array([0., 0., thrust])
        torque_body = np.array([roll_torque, pitch_torque, yaw_torque])

        # linear
        thrust_world = rotate_vector(rot, thrust_body)
        drag_force_world = (-self.k_linear_drag*Earth.density(pos)*np.linalg.norm(vel))*vel

        accel = Earth.gravity + (thrust_world + drag_force_world) / self.mass

        self.accel_body_state[:] = rotate_vector(quaternion_conjugate(rot), accel)

        # angular
        omega_quat = np.array([0., ang_vel[0], ang_vel[1], ang_vel[2]])
        d_rot = 0.5 * quaternion_multiply(rot, omega_quat)

        drag_torque = (-self.k_rot_drag * np.linalg.norm(ang_vel)) * ang_vel
        coriolis_torque = np.cross(ang_vel, self.moi * ang_vel)
        ang_accel = (torque_body + drag_torque - coriolis_torque) / self.moi

        return np.concatenate([
            vel,   # dPos
            accel, # dVel

            d_rot,     # dRot
            ang_accel, # dAngVel
        ])

    def linearize(self, x_eq:np.ndarray, u_eq:np.ndarray, eps=1e-6) -> list[np.ndarray]:
        def dynamics(x, u):
            pos = x[0:3]
            vel = x[3:6]
            rot = quaternion_from_euler(*x[6:9])
            ang_vel = x[9:12]

            S = self.dynamics(np.concatenate((pos, vel, rot, ang_vel)), u)

            vel = S[0:3]
            accel = S[3:6]
            ang_accel = S[10:13]

            return np.concatenate([vel, accel, ang_vel, ang_accel])

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
        dt = AvionicsConfig.control_sampling*AvionicsConfig.interval

        C = np.eye(12)
        D = np.zeros((12, 4))
        Ad, Bd, Cd, Dd, _ = cont2discrete( # type: ignore
            (A, B, C, D),
            dt,
            method='zoh'
        )

        return (Ad, Bd)

    def update(self, dt, ut):
        thrust = 0.
        roll_torque = 0.
        pitch_torque = 0.
        yaw_torque = 0.

        for rotor in self.rotors:
            rotor.update(dt)

            torque_body = np.cross(rotor.pos, np.array([0., 0., rotor.thrust]))

            thrust += rotor.thrust

            roll_torque += torque_body[0]
            pitch_torque += torque_body[1]
            yaw_torque += rotor.torque

        # step solver
        self.state = self.solver.step(dt, np.array([thrust, roll_torque, pitch_torque, yaw_torque]))

        # normalize rotation quat
        norm = np.linalg.norm(self.rotation)
        if norm > 1e-6:
            self.rotation /= norm
        else:
            self.rotation = np.array([1.0, 0.0, 0.0, 0.0])

        # update sensors
        for sensor in self.sensors:
            sensor.update(ut)


    def add_rotor(self, rotor:Rotor):
        self.rotors.append(rotor)

        self.available_thrust = 0.
        self.available_torque = np.array([0., 0., 0.]) # r, p, y

        for _rotor in self.rotors:
            _max_thrust = _rotor.kt * (_rotor.max_ang_vel ** 2)
            _max_yaw_torque = _rotor.b * (_rotor.max_ang_vel ** 2)

            self.available_thrust += _max_thrust

            torque_body = np.cross(_rotor.pos, np.array([0., 0., _max_thrust]))

            yaw_torque_body = np.array([0., 0., _max_yaw_torque]) # reaction torque

            self.available_torque += np.abs(torque_body) + np.abs(yaw_torque_body)

    def add_sensor(self, sensor:Sensor):
        self.sensors.append(sensor)
        return sensor

    def apply_control_input(self, u:np.ndarray):
        thrust, roll_torque, pitch_torque, yaw_torque = u

        # command
        for i, force in enumerate(self.MMA(thrust, roll_torque, pitch_torque, yaw_torque)):
            self.rotors[i].control = self.force_to_throttle(force, self.rotors[i])

    # motor mixing algorithm (without priority)
    def MMA(self, thrust:float, roll_torque:float, pitch_torque:float, yaw_torque:float) -> list[float]:
        delta_roll = roll_torque / (4 * self.size)
        delta_pitch = pitch_torque / (4 * self.size)
        delta_yaw = yaw_torque / (4 * RotorConfig.b_kt_ratio)

        thrust /= 4.0

        f_fr = thrust - delta_pitch + delta_yaw + delta_roll
        f_fl = thrust - delta_pitch - delta_yaw - delta_roll
        f_br = thrust + delta_pitch - delta_yaw + delta_roll
        f_bl = thrust + delta_pitch + delta_yaw - delta_roll

        return [f_fr, f_fl, f_br, f_bl]

    @staticmethod
    def force_to_throttle(force:float, rotor:Rotor) -> float:
        max_thrust = rotor.kt * (rotor.max_ang_vel**2)

        f_clip = np.clip(force, 0.0, max_thrust)

        return np.sqrt(f_clip / max_thrust)

    @property
    def model_state(self) -> np.ndarray:
        # [x, y, z, x', y', z', r, p, y, r', p', y']
        x, y, z, vx, vy, vz, qw, qx, qy, qz, v_roll, v_pitch, v_yaw = self.state
        roll, pitch, yaw = quaternion_to_euler(np.array([qw, qx, qy, qz]))

        return np.array([
            x, y, z,
            vx, vy, vz,
            roll, pitch, yaw,
            v_roll, v_pitch, v_yaw
        ])

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
    def angular_velocity(self) -> np.ndarray:
        return self.state[10:13]
    @angular_velocity.setter
    def angular_velocity(self, value:np.ndarray):
        self.state[10:13] = value

if __name__ == "__main__":
    from .rotor import Rotor
    from .sensors import Accelerometer, Gyroscope, GPS, Barometer
    from .config import DroneConfig, RotorConfig, InitialState

    quadcopter = Drone(DroneConfig.mass, DroneConfig.moi, DroneConfig.size)
    quadcopter.position = InitialState.s0
    quadcopter.velocity = InitialState.v0

    quadcopter.rotation = InitialState.q0
    quadcopter.angular_velocity = InitialState.w0

    rotor_kt = 1.e-6
    rotor_b = 2.e-8
    rotor_rpm = 20_000
    rotor_tau = 100e-3

    rotor_1 = Rotor(
        np.array([0.2, 0., 0.]),
        rotor_kt,
        rotor_b,
        rotor_rpm,
        True,
        time_constant=rotor_tau
    )

    rotor_2 = Rotor(
        np.array([-0.2, 0., 0.]),
        rotor_kt,
        rotor_b,
        rotor_rpm,
        False,
        time_constant=rotor_tau
    )

    rotor_1.control = 1.0
    rotor_2.control = 1.0

    quadcopter.add_rotor(rotor_1)
    quadcopter.add_rotor(rotor_2)

    sensor_acc = quadcopter.add_sensor(Accelerometer(quadcopter.accel_body_state))
    sensor_gyro = quadcopter.add_sensor(Gyroscope(quadcopter.state))
    sensor_gps = quadcopter.add_sensor(GPS(quadcopter.state))
    sensor_baro = quadcopter.add_sensor(Barometer(quadcopter.state))

    dt = 0.05
    for i in range(100):
        t = i*dt
        quadcopter.update(dt, t)

        # print(quadcopter.accel_body_state, sensor_acc)
        # print(quadcopter.angular_velocity, sensor_gyro)
        # print(quadcopter.position, sensor_gps)
        # print(quadcopter.position[2], sensor_baro)
