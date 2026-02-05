import numpy as np

# --- 1. CẤU HÌNH ROBOT ---
class UR10eParams:
    """Lưu trữ thông số DH và độ dài link của UR10e"""
    def __init__(self):
        self.d1 = 0.181
        #self.d4 = 0.1639
        self.d4 = 0.174
        self.a2 = 0.613
        self.a3 = 0.571
        #self.d5 = 0.1157
        self.d5 = 0.120 
        #self.d6 = 0.0922
        self.d6 = 0.117

# --- 2. CORE IK SOLVER (GIỮ NGUYÊN LOGIC CỦA BẠN) ---
def solve_ik_custom(x_tar, y_tar, z_tar, params=None):
    if params is None: params = UR10eParams()

    x = x_tar + params.d5 * np.cos(np.arctan2(y_tar,x_tar))
    y = y_tar + params.d5 * np.sin(np.arctan2(y_tar,x_tar))
    z = z_tar + params.d6

    # --- GIẢI 3 KHỚP ĐẦU (Position) ---
    rho = np.sqrt(x**2 + y**2)
    val_acos_q1 = np.clip(params.d4 / rho, -1, 1)
    q1 = np.arccos(val_acos_q1) + np.arctan2(y, x) + np.pi/2
    
    r_sq = x**2 + y**2 - params.d4**2
    h = z - params.d1
    
    val_D = (r_sq + h**2 + params.a2**2 - params.a3**2) / (2 * params.a2 * np.sqrt(r_sq + h**2))
    val_D = np.clip(val_D, -1, 1)
    q2 = np.pi/2 - np.arctan2(h, np.sqrt(r_sq)) - np.arccos(val_D)
    
    val_D3 = (r_sq + h**2 - params.a2**2 - params.a3**2) / (2 * params.a2 * params.a3)
    val_D3 = np.clip(val_D3, -1, 1)
    q3 = np.arccos(val_D3)

    # --- GIẢI 3 KHỚP CỔ TAY (Orientation - Nose Down) ---
    C1, S1 = np.cos(q1), np.sin(q1)
    C23, S23 = np.cos(q2 + q3), np.sin(q2 + q3)
    
    R03 = np.array([
        [C1*C23, -C1*S23,  S1],
        [S1*C23, -S1*S23, -C1],
        [S23,     C23,     0]
    ])
    
    R_target = np.array([[1, 0, 0], [0, 1, 0], [0, 0, -1]])
    R36 = R03.T @ R_target
    
    q5 = np.arccos(np.clip(R36[2, 2], -1, 1))
    q4 = np.arctan2(R36[1, 2], R36[0, 2])
    q6 = np.arctan2(R36[2, 1], -R36[2, 0])

    return np.array([q1, q2, q3, q4, q5, q6])

# --- 3. CLASS SINH QUỸ ĐẠO & TÍNH VẬN TỐC (CƠ CHẾ BẠN CẦN) ---
class TrajectoryGenerator:
    def __init__(self, dt=0.01):
        """
        dt: Thời gian lấy mẫu (Sampling time) của vòng điều khiển. 
            Cần khớp với dt trong MuJoCo/Simulink/Controller.
        """
        self.dt = dt
        self.params = UR10eParams()

    def generate_circle(self, center, radius, z_height, duration):
        """
        Sinh ra toàn bộ list q và dq cho quỹ đạo hình tròn.
        
        Input:
            center: tuple (cx, cy)
            radius: bán kính (m)
            z_height: độ cao z cố định (m)
            duration: tổng thời gian chạy hết vòng tròn (giây)
            
        Output:
            q_traj: numpy array shape (N, 6) -> Chứa vị trí các khớp
            dq_traj: numpy array shape (N, 6) -> Chứa vận tốc các khớp
        """
        num_steps = int(duration / self.dt)
        theta = np.linspace(0, 2*np.pi, num_steps)
        
        q_list = []
        dq_list = []
        q_prev = None
        
        cx, cy = center

        for k in range(num_steps):
            # 1. Tạo tọa độ XYZ tiếp theo
            x = cx + radius * np.cos(theta[k])
            y = cy + radius * np.sin(theta[k])
            z = z_height
            
            # 2. Giải IK để lấy q hiện tại
            q_curr = solve_ik_custom(x, y, z, self.params)
            
            # 3. Tính vận tốc dq (Finite Difference)
            if q_prev is None:
                # Bước đầu tiên vận tốc bằng 0 (giả sử đứng yên)
                dq_curr = np.zeros(6)
            else:
                # v = (x2 - x1) / dt
                dq_curr = (q_curr - q_prev) / self.dt
            
            # 4. Lưu vào list
            q_list.append(q_curr)
            dq_list.append(dq_curr)
            
            # Cập nhật q cũ
            q_prev = q_curr
            
        return np.array(q_list), np.array(dq_list)