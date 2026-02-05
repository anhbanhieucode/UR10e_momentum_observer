import matplotlib.pyplot as plt
import numpy as np

def plot_comparison(logs, config):
    """Vẽ so sánh Torque ước lượng (FDI) và Ngoại lực thực tế (Inject)"""
    t = np.array(logs['time'])
    r = np.array(logs['r_fdi'])
    f_real = np.array(logs['force_real'])
    
    target_joint = config['joint']
    
    fig, axs = plt.subplots(3, 2, figsize=(15, 10))
    fig.suptitle(f'Momentum Observer (Impact on Joint {target_joint})', fontsize=16)
    
    joints = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    
    for i in range(6):
        ax = axs[i//2, i%2]
        ax.set_ylim(-60, 60)
        
        # Đường màu ĐỎ: Giá trị ước lượng từ FDI
        ax.plot(t, r[:, i], color='red', linewidth=2, label='FDI Estimate (r)')
        
        # Đường màu XANH (Nét đứt): Lực thực tế từ error.py
        ax.plot(t, f_real[:, i], color='blue', linestyle='--', linewidth=2, alpha=0.8, label='Real Force (Input)')
        
        ax.set_title(f'Joint {i}: {joints[i]}')
        ax.set_ylabel('Torque (Nm)')
        ax.grid(True, linestyle=':', alpha=0.6)
        
        # Highlight khớp bị tác động
        if i == target_joint:
            ax.legend(loc='upper right')
            ax.set_facecolor('#fffdc0') # Nền vàng nhạt

    plt.tight_layout()
    plt.show()

def plot_IK(logs):
    """Vẽ so sánh góc khớp (Joint Space Tracking)"""
    t = np.array(logs['time'])
    q_ref = np.array(logs['q_ref'])
    q_real = np.array(logs['q_real'])
    
    fig, axs = plt.subplots(3, 2, figsize=(15, 10))
    fig.suptitle('CTC Trajectory Tracking in Joint Space', fontsize=16)
    
    joints = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    
    for i in range(6):
        ax = axs[i//2, i%2]
        
        ax.plot(t, q_ref[:, i], 'k--', linewidth=2, label='Reference')
        ax.plot(t, q_real[:, i], 'b-', linewidth=1.5, label='Real')
        
        ax.set_title(f'Joint {i}: {joints[i]}')
        ax.set_ylabel('Angle (rad)')
        ax.set_xlabel('Time (s)')
        ax.grid(True, linestyle=':', alpha=0.6)
        
        rmse = np.sqrt(np.mean((q_ref[:, i] - q_real[:, i])**2))
        ax.text(0.05, 0.9, f'RMSE: {rmse:.4f}', transform=ax.transAxes, 
                bbox=dict(facecolor='white', alpha=0.8))
        
        if i == 0: ax.legend(loc='lower right')

    plt.tight_layout()
    plt.show()

def plot_trajectory_comparison(logs):
    """Vẽ so sánh quỹ đạo End-Effector (Task Space Tracking)"""
    xyz_ref = np.array(logs['xyz_ref'])
    xyz_real = np.array(logs['xyz_real'])
    
    # 1. Vẽ 3D
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    
    ax.plot(xyz_ref[:, 0], xyz_ref[:, 1], xyz_ref[:, 2], 'r--', linewidth=2, label='Ref')
    ax.plot(xyz_real[:, 0], xyz_real[:, 1], xyz_real[:, 2], 'b-', linewidth=2, label='Real')
    
    ax.set_title('Circular Trajectory End Effector Tracking')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.legend()
    
    # Box aspect
    all_coords = np.vstack((xyz_ref, xyz_real))
    ax.set_box_aspect((np.ptp(all_coords[:, 0]), np.ptp(all_coords[:, 1]), np.ptp(all_coords[:, 2])))

    # 2. Vẽ sai số
    ax2 = fig.add_subplot(1, 2, 2)
    t = np.array(logs['time'])
    min_len = min(len(xyz_ref), len(xyz_real))
    error = np.linalg.norm(xyz_ref[:min_len] - xyz_real[:min_len], axis=1)
    
    ax2.plot(t[:min_len], error * 1000, 'k-', linewidth=1.5)
    ax2.set_title('Tracking Error (mm)')
    ax2.grid(True)
    
    mean_error = np.mean(error) * 1000
    ax2.text(0.05, 0.9, f'Mean: {mean_error:.2f} mm', transform=ax2.transAxes, bbox=dict(facecolor='yellow', alpha=0.3))

    plt.tight_layout()
    plt.show()