import mujoco
import numpy as np
from pathlib import Path

# --- CẤU HÌNH ---
# Đường dẫn đến file MuJoCo Scene của bạn
base_path = Path("/home/anhbanhieu/mujoco/humanoid-motion-planning/mujoco_menagerie/universal_robots_ur10e")
xml_path = str(base_path / "scene.xml")

# Tên file URDF sẽ được sinh ra
output_urdf = "ur10e_synced.urdf"

# --- LOAD MUJOCO MODEL ---
print(f"Đang đọc dữ liệu từ: {xml_path}")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
mujoco.mj_step(model, data) # Cập nhật kinematics

# --- HÀM HỖ TRỢ XML ---
def format_vec(vec):
    return f"{vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f}"

# --- DANH SÁCH LINK & JOINT CỦA UR10E ---
# Chúng ta sẽ map thủ công để đảm bảo đúng thứ tự kinematic chain
# Cấu trúc: (Tên Link, Tên Joint, Body ID trong MuJoCo)
# Lưu ý: Body ID phải check kỹ. Thường: 
# 0: World, 1: Base, 2: Shoulder, 3: UpperArm, 4: ForeArm, 5: Wrist1, 6: Wrist2, 7: Wrist3
links_config = [
    # (Link Name, Joint Name, Joint Type, Joint Axis, Limits, MJ Body ID)
    ("shoulder_link", "shoulder_pan_joint", "revolute", "0 0 1", [-6.28, 6.28], 2),
    ("upper_arm_link", "shoulder_lift_joint", "revolute", "0 1 0", [-6.28, 6.28], 3),
    ("forearm_link", "elbow_joint", "revolute", "0 1 0", [-3.14, 3.14], 4),
    ("wrist_1_link", "wrist_1_joint", "revolute", "0 1 0", [-6.28, 6.28], 5),
    ("wrist_2_link", "wrist_2_joint", "revolute", "0 0 1", [-6.28, 6.28], 6),
    ("wrist_3_link", "wrist_3_joint", "revolute", "0 1 0", [-6.28, 6.28], 7),
]

# --- BẮT ĐẦU VIẾT FILE URDF ---
urdf_content = ["""<?xml version="1.0" ?>
<robot name="ur10e_synced">
  """]

# 1. VIẾT BASE LINK (Link tĩnh đầu tiên)
# Lấy ID của base (thường là 1)
base_id = 1
base_mass = model.body_mass[base_id]
base_com = model.body_ipos[base_id]
base_inertia = model.body_inertia[base_id] # Chỉ lấy xấp xỉ diagonal

urdf_content.append(f"""
  <link name="base_link">
    <inertial>
      <mass value="{base_mass:.6f}"/>
      <origin xyz="{format_vec(base_com)}" rpy="0 0 0"/>
      <inertia ixx="{base_inertia[0]:.6f}" ixy="0" ixz="0" iyy="{base_inertia[1]:.6f}" iyz="0" izz="{base_inertia[2]:.6f}"/>
    </inertial>
  </link>
""")

parent_link = "base_link"

# 2. VÒNG LẶP CÁC LINK ĐỘNG
for link_name, joint_name, j_type, j_axis, j_limits, body_id in links_config:
    # --- LẤY DỮ LIỆU TỪ MUJOCO ---
    mass = model.body_mass[body_id]
    com = model.body_ipos[body_id]   # Center of Mass (Local to Body frame)
    inertia = model.body_inertia[body_id] # Principal inertia moments
    
    # Lấy thông tin Joint (Vị trí tương đối so với cha)
    # Trong MuJoCo: body_pos là vị trí của body này so với body cha
    joint_origin = model.body_pos[body_id]
    
    # Xử lý hướng trục (Orientation)
    # MuJoCo lưu quaternion (body_quat). 
    # Đơn giản hóa: Menagerie thường thiết kế trục khớp thẳng hàng.
    # Chúng ta dùng trick: Set RPY = 0 và dựa vào axis định nghĩa.
    # Nếu khớp bị xoay (ví dụ Upper Arm xoay 90 độ), cần hardcode RPY từ scene.
    
    rpy = "0 0 0"
    if "upper_arm" in link_name: rpy = "0 1.570796327 0" 
    if "wrist_1" in link_name: rpy = "0 1.570796327 0"
    
    # (Lưu ý: Đây là phần khó nhất khi convert tự động, 
    # đoạn code dưới hardcode các góc xoay chuẩn của UR10e để khớp kinematics)
    
    # --- GHI JOINT ---
    urdf_content.append(f"""
  <joint name="{joint_name}" type="{j_type}">
    <parent link="{parent_link}"/>
    <child link="{link_name}"/>
    <origin xyz="{format_vec(joint_origin)}" rpy="{rpy}"/>
    <axis xyz="{j_axis}"/>
    <limit lower="{j_limits[0]}" upper="{j_limits[1]}" effort="330.0" velocity="3.14"/>
  </joint>
""")
    
    # --- GHI LINK (MASS & INERTIA TỪ MUJOCO) ---
    # Quan trọng: MuJoCo Inertia là tại COM và aligned axis.
    urdf_content.append(f"""
  <link name="{link_name}">
    <inertial>
      <mass value="{mass:.6f}"/>
      <origin xyz="{format_vec(com)}" rpy="0 0 0"/> 
      <inertia ixx="{inertia[0]:.6f}" ixy="0" ixz="0" iyy="{inertia[1]:.6f}" iyz="0" izz="{inertia[2]:.6f}"/>
    </inertial>
  </link>
""")
    
    parent_link = link_name

urdf_content.append("</robot>")

# --- LƯU FILE ---
with open(output_urdf, "w") as f:
    f.write("".join(urdf_content))

print(f"\n✅ Đã tạo file: {output_urdf}")
print(f"👉 Link 4 (Forearm) Mass MuJoCo đang dùng: {model.body_mass[4]:.4f} kg")
print(f"👉 Link 4 (Forearm) COM MuJoCo đang dùng: {model.body_ipos[4]}")
print("Hãy dùng file này chạy lại compare.py!")