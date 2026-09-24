import numpy as np

from utils import quaternion_multiply, quaternion_conjugate, rotate_vector

from core.rotor import Rotor
from core.earth import Earth
from core.vehicle import Vehicle

from config import RotorConfig

class Quadcopter(Vehicle):
    def __init__(self, mass:float, moi:np.ndarray, size:float):
        super().__init__(mass, moi)

        self.accel_body_state = np.array([0., 0., 0.])

        self.size = size

        # drag
        self.k_linear_drag = 0.4 * (self.size**2)
        self.k_rot_drag = 0.01

    # override
    def u(self, force_body:np.ndarray, torque_body:np.ndarray) -> np.ndarray:
        return [force_body[2], torque_body[0], torque_body[1], torque_body[2]]

    # override
    def dynamics(self, x:np.ndarray, u:np.ndarray) -> np.ndarray:
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
        max_thrust = rotor.kt * (rotor.max_rotation**2)

        return np.sqrt(np.clip(force, 0.0, max_thrust) / max_thrust)

    def apply_control_input(self, u:np.ndarray):
        thrust, roll_torque, pitch_torque, yaw_torque = u

        # command
        for i, force in enumerate(self.MMA(thrust, roll_torque, pitch_torque, yaw_torque)):
            self.propulsions[i].throttle = self.force_to_throttle(force, self.propulsions[i])