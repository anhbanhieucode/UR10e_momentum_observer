import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
from pathlib import Path

# --- LOCAL IMPORTS ---
from controllers import PIDController, FDI
from visualizer import plot_comparison, plot_IK, plot_trajectory_comparison, plot_residuals
from ur10e_dynamics import get_dynamics
from IK_ur10e import TrajectoryGenerator
from error import inject_force

# --- CẤU HÌNH PATH ---
sys.path.insert(0, str(Path(__file__).parent))
base = Path(__file__).parent.parent
xml_path = str(base / "UR10e_observer/universal_robots_ur10e/scene.xml")

def main():
    # 1. CẤU HÌNH
    residual_threshold = 25.0 
    is_emergency_stop = False 
    q_collision = None  # Lưu vị trí tại thời điểm va chạm

    print(f"Loading MuJoCo: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    DT = model.opt.timestep

    # 2. KHỞI TẠO CONTROLLER & PLANNER
    pid = PIDController(
        kp=[100, 100, 100, 800, 900, 1000], 
        ki=[1, 1, 1, 1, 1, 1], 
        kd=[20, 20, 20, 20, 20, 20], 
        dt=DT
    )

    traj_gen = TrajectoryGenerator(dt=DT)
    
    CENTER = (0.1, -0.45)
    RADIUS = 0.25
    Z_HEIGHT = 0.3
    DURATION = 5.0
    
    q_data, dq_data = traj_gen.generate_circle(
        center=CENTER, radius=RADIUS, z_height=Z_HEIGHT, duration=DURATION
    )

    # FDI Params
    damping_params = np.array([10.0, 10.0, 10.0, 2.0, 2.0, 2.0])
    fdi_state = {'integral': np.zeros(6), 'r': np.zeros(6), 'M_prev': None}
    K_fdi = np.diag([30.0] * 6)

    # Logger
    logs = {'time': [], 'r_fdi': [], 'force_real': [], 'q_ref':[], 'q_real':[], 'xyz_ref': [], 'xyz_real': []}

    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    use_site = True if tcp_id != -1 else False

    q_home = np.array([0, -1.57, 1.57, 0, 0, 0])
    data.qpos[:6] = q_home

    theta_arc = np.linspace(0, 2*np.pi, len(q_data))
    SIM_DURATION = 7.0
    SKIP_STEPS = 100  # Bỏ qua nhiễu khởi động
    step_count = 0    

    print(f"\n SIMULATION START ({SIM_DURATION}s)...")

    # 3. VÒNG LẶP MÔ PHỎNG
    with mujoco.viewer.launch_passive(model, data) as viewer:
        traj_idx = 0
        while viewer.is_running() and data.time < SIM_DURATION:
            step_count += 1
            t = data.time
            q_curr = data.qpos[:6]
            dq_curr = data.qvel[:6]
            
            # Tính toán Động lực học (Dùng hàm tự tính toán)
            q_real_dyn = q_curr + np.array([0.0, np.pi/2, 0.0, 0.0, 0.0, 0.0])
            M, C_matrix, G = get_dynamics(q_real_dyn, dq_curr)
            C_vector = C_matrix @ dq_curr

            if not is_emergency_stop:
                # A. Trajectory Reference
                if traj_idx < len(q_data):
                    ur_offset = np.array([0, -np.pi/2 , 0, -np.pi/2, 0, 0])
                    q_ref = q_data[traj_idx] + ur_offset
                    dq_ref = dq_data[traj_idx]
                    traj_idx += 1
                else:
                    q_ref = q_data[-1] + ur_offset
                    dq_ref = np.zeros(6)

                # B. Control Law (CTC)
                tau_pid = pid.update(q_curr, dq_curr, q_ref, dq_ref)
                tau_cmd = M @ tau_pid + C_vector + G
                
                # C. FDI Observer
                r_est = FDI(tau_cmd, M, C_vector, G, dq_curr, DT, fdi_state, K_fdi, damping_params)

                # D. Kiểm tra va chạm sau thời gian SKIP
                if step_count >= SKIP_STEPS:
                    if np.any(np.abs(r_est) > residual_threshold):
                        print(f"!!! COLLISION DETECTED at {t:.2f}s !!!")
                        is_emergency_stop = True
                        q_collision = q_curr.copy()  # Chốt vị trí va chạm
                        data.qvel[:6] = 0.0  # Triệt tiêu vận tốc tức thì
            
            if is_emergency_stop:
                # TRẠNG THÁI DỪNG TUYỆT ĐỐI: Khóa cứng vị trí bằng PID
                tau_lock = pid.update(q_curr, dq_curr, q_collision, np.zeros(6))
                tau_cmd = M @ tau_lock + G 
                r_est = fdi_state['r']

            # Thực thi điều khiển
            data.ctrl[:6] = tau_cmd
            mujoco.mj_step(model, data)
            viewer.sync()

            # --- H. Logging ---
            if step_count >= SKIP_STEPS:
                idx_log = min(traj_idx, len(theta_arc)-1)
                x_ref = CENTER[0] + RADIUS * np.cos(theta_arc[idx_log])
                y_ref = CENTER[1] + RADIUS * np.sin(theta_arc[idx_log])
                
                logs['xyz_ref'].append([x_ref, y_ref, Z_HEIGHT])
                logs['xyz_real'].append(data.site_xpos[tcp_id].copy() if use_site else data.xpos[tcp_id].copy())
                logs['time'].append(t)
                logs['q_ref'].append(q_ref.copy() if not is_emergency_stop else q_collision.copy())
                logs['q_real'].append(q_curr.copy())
                logs['r_fdi'].append(r_est.copy())

    print(" Simulation Finished! Plotting results...")
    plot_residuals(logs, threshold=residual_threshold)
    plot_IK(logs)
    plot_trajectory_comparison(logs)

if __name__ == "__main__":
    main()