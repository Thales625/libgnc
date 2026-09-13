import numpy as np

def quaternion_conjugate(q:np.ndarray) -> np.ndarray:
    return q * np.array([1, -1, -1, -1])

def quaternion_multiply(q1:np.ndarray, q2:np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return v

    q_unit = q / norm

    # vec as quat
    v_quat = np.array([0., v[0], v[1], v[2]])

    q_conj = quaternion_conjugate(q)

    # v' = q_unit * v_quat * q_conj
    return quaternion_multiply(quaternion_multiply(q_unit, v_quat), q_conj)[1:]

def quaternion_to_euler(quat:np.ndarray) -> np.ndarray:
    w, x, y, z = quat

    # roll (x-axis)
    sinr_cosr = 2.0 * (w * x + y * z)
    cosr_cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosr, cosr_cosr)

    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * (np.pi / 2.0)
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis)
    siny_cosy = 2.0 * (w * z + x * y)
    cosy_cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosy, cosy_cosy)

    return np.array([roll, pitch, yaw])

def quaternion_from_euler(roll: float, pitch: float, yaw: float, deg_to_rad=False) -> np.ndarray:
    if deg_to_rad:
        roll *= np.pi / 180.0
        pitch *= np.pi / 180.0
        yaw *= np.pi / 180.0

    # half angles
    hr = roll  * 0.5 # X
    hp = pitch * 0.5 # Y
    hy = yaw   * 0.5 # Z

    cr, sr = np.cos(hr), np.sin(hr)
    cp, sp = np.cos(hp), np.sin(hp)
    cy, sy = np.cos(hy), np.sin(hy)

    # yaw-pitch-roll
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    quat = np.array([w, x, y, z])
    return quat / np.linalg.norm(quat)
