"""PID ve LQR denetleyicileri."""
import numpy as np
from scipy.linalg import solve_continuous_are

from dynamics import Params, linearized_matrices


class PID:
    """Çubuk açısı üzerinde PID. Opsiyonel olarak araba konumu için PD terimi.

    u = Kp*th + Ki*int(th) + Kd*th_dot + (kx*x + kv*x_dot)
    """

    def __init__(self, kp=60.0, ki=0.0, kd=10.0, kx=0.0, kv=0.0, dt=0.005):
        self.kp, self.ki, self.kd, self.kx, self.kv, self.dt = kp, ki, kd, kx, kv, dt
        self.integral = 0.0

    def __call__(self, s):
        x, xd, th, thd = s
        self.integral += th * self.dt
        return self.kp * th + self.ki * self.integral + self.kd * thd + self.kx * x + self.kv * xd


class LQR:
    """Doğrusallaştırılmış sistem için LQR: u = -K s, K = R^-1 B^T P (Riccati)."""

    def __init__(self, p: Params, Q=None, R=None):
        A, B = linearized_matrices(p)
        self.Q = np.diag([1.0, 1.0, 10.0, 1.0]) if Q is None else np.asarray(Q)
        self.R = np.array([[0.1]]) if R is None else np.asarray(R)
        P = solve_continuous_are(A, B, self.Q, self.R)
        self.K = (np.linalg.inv(self.R) @ B.T @ P).flatten()

    def __call__(self, s):
        return float(-self.K @ s)
