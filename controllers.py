import numpy as np

# ==========================================
# PID CONTROLLER
# ==========================================
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = np.diag(kp)
        self.ki = np.diag(ki)
        self.kd = np.diag(kd)
        self.dt = dt
        self.integral = np.zeros(6)

    def update(self, q_curr, dq_curr, q_ref, dq_ref):
        error = q_ref - q_curr
        d_error = dq_ref - dq_curr
        self.integral += error * self.dt
        
        # Công thức PID: Kp*e + Ki*int(e) + Kd*de
        tau = self.kp @ error + self.ki @ self.integral + self.kd @ d_error
        return tau

# ==========================================
# FDI (MOMENTUM OBSERVER)
# ==========================================
def FDI(tau, M, C, G, dq, dt, state, K, damping_params):
    """
    Momentum-based Observer để ước lượng ngoại lực/va chạm.
    """
    integral = state['integral']
    r_old = state['r']
    M_prev = state['M_prev']

    if M_prev is None: M_prev = M.copy()

    # Momentum p = M * dq
    p = M @ dq
    M_dot = (M - M_prev) / dt

    # Tính toán ma sát (nếu có mô hình ma sát)
    tau_friction = damping_params * dq 
    
    # Dynamics terms: alpha = G + C*dq - M_dot*dq + Friction
    # Lưu ý: Trong lý thuyết gốc De Luca, alpha = g(q) - C^T(q,dq)*dq
    # Ở đây dùng dạng biến đổi tương đương cho discrete time
    alpha = G + C + tau_friction - (M_dot @ dq)

    # Đạo hàm của dư lượng r
    z_dot = tau - alpha - r_old
    integral += z_dot * dt

    # Dư lượng r (Residual) - Ước lượng của Torque ngoại lực
    r_new = K @ (integral - p)

    # Cập nhật state
    state['integral'] = integral
    state['r'] = r_new
    state['M_prev'] = M.copy()

    # Trả về -r_new vì r thường ngược dấu với ngoại lực trong một số quy ước
    return -r_new