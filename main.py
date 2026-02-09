import mujoco
import mujoco.viewer
import numpy as np
import time
import sys
from pathlib import Path

# --- LOCAL IMPORTS ---
from controllers import PIDController, FDI
from visualizer import plot_comparison, plot_IK, plot_trajectory_comparison
from ur10e_dynamics import get_dynamics
from IK_ur10e import TrajectoryGenerator
from error import inject_force

# --- CẤU HÌNH PATH ---
sys.path.insert(0, str(Path(__file__).parent))
base = Path(__file__).parent.parent
xml_path = str(base / "UR10e_observer/universal_robots_ur10e/scene.xml")

def main():
    # 1. CẤU HÌNH

    collision_config = {
        'joint': 1,      # Khớp va chạm
        'start': 3.0,    # Thời gian bắt đầu
        'end': 5.0,      # Thời gian kết thúc
        'mag': 20.0       # Lực tác động (Nm)
    }


    print(f"Loading MuJoCo: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    DT = model.opt.timestep

    # 2. KHỞI TẠO CONTROLLER & PLANNER
    # PID Gains (CTC Tuning: Kp thấp hơn PID thuần, Kd vừa phải)
    pid = PIDController(
        kp=[100, 100, 100, 800, 900, 1000], 
        ki=[1, 1, 1, 1, 1, 1], 
        kd=[20, 20, 20, 20, 20, 20], 
        dt=DT
    )

    traj_gen = TrajectoryGenerator(dt=DT)
    
    # Sinh quỹ đạo hình tròn
    CENTER = (0.4, -0.3)
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

    # Lấy ID của End-Effector (attachment_site)
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    if tcp_id == -1:
        print(" Warning: Không tìm thấy 'attachment_site', dùng body 'wrist_3_link'")
        tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wrist_3_link")
        use_site = False
    else:
        use_site = True

    # Set Home Pose
    q_home = np.array([0, -1.57, 1.57, 0, 0, 0])
    data.qpos[:6] = q_home

    # Tính lại theta cho việc logging
    theta_arc = np.linspace(0, 2*np.pi, len(q_data))
    
    SIM_DURATION = 30.0
    print(f"\n SIMULATION START ({SIM_DURATION}s)...")

    # 3. VÒNG LẶP MÔ PHỎNG
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time < SIM_DURATION:
            for i in range(len(q_data)):
                t = data.time
                
                # A. Trajectory Reference
                ur_offset = np.array([0, -np.pi/2 , 0, -np.pi/2, 0, 0])
                q_ref = q_data[i] + ur_offset
                dq_ref = dq_data[i]

                # B. Inject Error (Ngoại lực)
                real_force_vector = inject_force(data, t, collision_config)

                # C. Feedback
                q_curr = data.qpos[:6]
                dq_curr = data.qvel[:6]
                
                # D. Dynamics Calculation (CTC)
                q_real_dyn = q_curr + np.array([0.0, np.pi/2, 0.0, 0.0, 0.0, 0.0])
                M, C_matrix, G = get_dynamics(q_real_dyn, dq_curr)
                C_vector = C_matrix @ dq_curr
                
                # E. Control Law (Computed Torque Control)
                tau_pid = pid.update(q_curr, dq_curr, q_ref, dq_ref)
                tau_cmd = M @ tau_pid + C_vector + G
                data.ctrl[:6] = tau_cmd
                
                # F. FDI Observer
                r_est = FDI(tau_cmd, M, C_vector, G, dq_curr, DT, fdi_state, K_fdi, damping_params)

                # G. Step Physics
                mujoco.mj_step(model, data)
                viewer.sync()

                # H. Logging
                # Log XYZ lý thuyết
                x_ref = CENTER[0] + RADIUS * np.cos(theta_arc[i])
                y_ref = CENTER[1] + RADIUS * np.sin(theta_arc[i])
                logs['xyz_ref'].append([x_ref, y_ref, Z_HEIGHT])

                # Log XYZ thực tế
                if use_site:
                    xyz_real = data.site_xpos[tcp_id].copy()
                else:
                    xyz_real = data.xpos[tcp_id].copy()
                logs['xyz_real'].append(xyz_real)

                logs['time'].append(t)
                logs['q_ref'].append(q_ref.copy())
                logs['q_real'].append(q_curr.copy())
                logs['r_fdi'].append(r_est.copy())
                logs['force_real'].append(real_force_vector.copy())
            
    print(" Simulation Finished! Plotting results...")
    plot_comparison(logs, collision_config)
    plot_IK(logs)
    plot_trajectory_comparison(logs)

if __name__ == "__main__":
    main()