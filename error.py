import numpy as np
import mujoco

def inject_force(data, t, config):
    """
    Hàm tiêm lỗi (Fault Injection) hoặc tạo va chạm giả lập.
    
    Args:
        data: Đối tượng mujoco.MjData (để tác động lực vào sim).
        t: Thời gian hiện tại.
        config: Dictionary chứa cấu hình xung lực {'joint', 'start', 'end', 'mag'}.
        
    Returns:
        real_force_vector (np.array): Vector lực thực tế đang tác động (để vẽ đồ thị so sánh).
    """
    
    # 1. Khởi tạo vector ngoại lực bằng 0 (cho 6 khớp)
    # Đây là vector Ground Truth (Lực thực tế)
    disturbance_torque = np.zeros(6)

    # 2. Lấy tham số từ config
    joint_idx = config.get('joint', 0)  # Khớp nào bị tác động
    t_start = config.get('start', 0)    # Thời gian bắt đầu
    t_end = config.get('end', 0)        # Thời gian kết thúc
    magnitude = config.get('mag', 0)    # Độ lớn lực (Nm)

    # 3. Kiểm tra logic thời gian (Tạo xung vuông)
    if t_start <= t <= t_end:
        # Gán giá trị lực vào đúng khớp
        disturbance_torque[joint_idx] = magnitude
        
        # --- QUAN TRỌNG: TÁC ĐỘNG VÀO MUJOCO ---
        # qfrc_applied: Applied generalized force. 
        data.qfrc_applied[:6] = disturbance_torque
        
    else:
        # Nếu hết thời gian va chạm, phải reset lực về 0
        # Nếu không reset, lực sẽ giữ nguyên mãi mãi
        data.qfrc_applied[:6] = np.zeros(6)

    # 4. Trả về vector lực để lưu vào log (phục vụ vẽ đồ thị)
    return disturbance_torque