import numpy as np

from utils import quaternion_multiply, quaternion_conjugate, rotate_vector

from core.rotor import Rotor
from core.earth import Earth
from core.vehicle import Vehicle

class Airplane(Vehicle):
    def __init__(self, mass:float, moi:np.ndarray):
        super().__init__(mass, moi)

        self.accel_body_state = np.array([0., 0., 0.])

        # params
        self.wing_area = 0.25 # m^2
        self.wing_span = 1.2  # m
        self.mean_chord = 0.2 # m

        self.CL0 = 0.3
        self.CL_alpha = 5.0

        self.CD0 = 0.03
        self.CD_k = 0.05

        self.Cm_alpha = -1.0
        self.Cm_elevator = -1.5

        self.Cl_aileron = 0.8
        self.Cn_rudder = 0.5

    # override
    def u(self, force_body:np.ndarray, torque_body:np.ndarray) -> np.ndarray:
        return [force_body[2], torque_body[0], torque_body[1], torque_body[2]]

    # override
    def dynamics(self, x:np.ndarray, u:np.ndarray) -> np.ndarray:
        pos = x[0:3]
        vel = x[3:6]
        rot = x[6:10]
        ang_vel = x[10:13]

        throttle, aileron, elevator, rudder = u
        thrust = throttle * self.propulsions[0].max_thrust

        # linear
        vel_body = rotate_vector(quaternion_conjugate(rot), vel)

        u_b = vel_body[0]
        v_b = vel_body[1]
        w_b = vel_body[2]

        V = np.linalg.norm(vel_body)
        alpha = np.arctan2(w_b, u_b + 1e-6)
        beta = np.arctan2(v_b, u_b + 1e-6)
        rho = Earth.density(pos)

        CL = self.CL0 + self.CL_alpha * alpha

        lift = 0.5 * rho * V**2 * self.wing_area * CL

        CD = self.CD0 + self.CD_k * CL**2

        drag = 0.5 * rho * V**2 * self.wing_area * CD

        force_body = np.array([
            -drag + thrust,
            0.0,
            -lift
        ])

        roll_moment = 0.5 * rho * V**2 * self.wing_area * self.wing_span * self.Cl_aileron * aileron
        Cm = self.Cm_alpha * alpha + self.Cm_elevator * elevator

        pitch_moment = 0.5 * rho * V**2 * self.wing_area * self.mean_chord * Cm
        yaw_moment = 0.5 * rho * V**2 * self.wing_area * self.wing_span * self.Cn_rudder * rudder

        force_world = rotate_vector(rot, force_body)

        accel = Earth.gravity + (force_world  / self.mass)

        self.accel_body_state[:] = rotate_vector(quaternion_conjugate(rot), accel)

        # angular
        torque_body = np.array([roll_moment, pitch_moment, yaw_moment])

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


    @staticmethod
    def force_to_throttle(force:float, rotor:Rotor) -> float:
        max_thrust = rotor.kt * (rotor.max_rotation**2)

        return np.sqrt(np.clip(force, 0.0, max_thrust) / max_thrust)

    def apply_control_input(self, u:np.ndarray):
        thrust, roll_torque, pitch_torque, yaw_torque = u

        self.propulsions[0].throttle = self.force_to_throttle(thrust, self.propulsions[0])