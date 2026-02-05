import mujoco
import pinocchio as pin
import numpy as np
import sys
from pathlib import Path

# --- CẤU HÌNH ĐƯỜNG DẪN ---
# 1. Đường dẫn đến file MuJoCo XML
# (Dựa trên đường dẫn bạn cung cấp trước đó)
base_path = Path("/home/anhbanhieu/mujoco/humanoid-motion-planning/mujoco_menagerie/universal_robots_ur10e")
xml_path = str(base_path / "scene.xml") # Hoặc ur10e.xml tuỳ bạn dùng file nào chính

# 2. Đường dẫn file Pinocchio URDF (File vừa tạo bằng script)
urdf_path = "ur10e_synced.urdf"

# --- LOAD MODEL ---
print(f"Loading MuJoCo: {xml_path}")
try:
    m_mj = mujoco.MjModel.from_xml_path(xml_path)
    d_mj = mujoco.MjData(m_mj)
except Exception as e:
    print(f"❌ Lỗi load MuJoCo: {e}")
    sys.exit(1)

print(f"Loading Pinocchio: {urdf_path}")
try:
    m_pin = pin.buildModelFromUrdf(urdf_path)
    d_pin = m_pin.createData()
except Exception as e:
    print(f"❌ Lỗi load Pinocchio: {e}")
    sys.exit(1)

print("-" * 50)
print(f"MuJoCo DoF: {m_mj.nv}")
print(f"Pinocchio DoF: {m_pin.nv}")

# Kiểm tra xem số khớp có khớp nhau không (Thường MuJoCo là 6, Pinocchio là 6)
if m_mj.nv != m_pin.nv:
    print("⚠️ CẢNH BÁO: Số bậc tự do không khớp! Chỉ so sánh 6 khớp đầu tiên.")

# --- VÒNG LẶP SO SÁNH ---
n_tests = 5
print(f"\nBắt đầu so sánh {n_tests} tư thế ngẫu nhiên...\n")

for i in range(n_tests):
    # 1. Tạo trạng thái ngẫu nhiên
    q = np.random.uniform(-np.pi, np.pi, 6)
    dq = np.zeros(6) # So sánh Gravity trước nên để vận tốc = 0
    
    # 2. Tính bằng MuJoCo
    d_mj.qpos[:6] = q
    d_mj.qvel[:6] = dq
    mujoco.mj_step(m_mj, d_mj) # Update kinematics
    
    # Lấy Gravity (qfrc_bias khi v=0)
    G_mj = d_mj.qfrc_bias[:6].copy()
    
    # Lấy Mass Matrix
    M_mj_full = np.zeros((m_mj.nv, m_mj.nv))
    mujoco.mj_fullM(m_mj, M_mj_full, d_mj.qM)
    M_mj = M_mj_full[:6, :6]
    
    # 3. Tính bằng Pinocchio
    pin.computeAllTerms(m_pin, d_pin, q, dq)
    G_pin = d_pin.g
    M_pin = d_pin.M
    
    # 4. Tính sai số
    diff_G = np.linalg.norm(G_mj - G_pin)
    diff_M = np.linalg.norm(M_mj - M_pin)
    
    print(f"Test {i+1}:")
    print(f"  - Sai số Gravity (Norm): {diff_G:.6f}")
    print(f"  - Sai số Mass Matrix (Norm): {diff_M:.6f}")
    
    # In chi tiết nếu sai số lớn
    if diff_G > 0.1:
        print("    -> 🔴 LỆCH GRAVITY LỚN!")
        print(f"       MJ:  {G_mj}")
        print(f"       PIN: {G_pin}")
    elif diff_M > 0.1:
        print("    -> 🔴 LỆCH MASS LỚN!")

print("-" * 50)
if diff_G < 0.05 and diff_M < 0.05:
    print("✅ KẾT LUẬN: MATCHED! Hai model tương đồng. Có thể dùng cho Observer.")
else:
    print("❌ KẾT LUẬN: MISMATCH! Cần kiểm tra lại file URDF (thường do base rotation).")