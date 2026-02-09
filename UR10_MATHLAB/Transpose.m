% ur10e_forward_kinematics_manual.m
% Tính toán Động học thuận UR10e (Modified DH)
% Phong cách: Tường minh, lấy tham số từ file cấu hình (ur10e_constants.m)
% --------------------------------------------------------------------

clear; clc;

% 1. KIỂM TRA VÀ LOAD FILE CẤU HÌNH
% ---------------------------------
if ~isfile('config.m')
    error('Lỗi: Không tìm thấy file "ur10e_constants.m". Vui lòng tạo file này trước!');
end
fprintf('Đang load thông số từ ur10e_constants.m...\n');
config = config(); % Gọi hàm config

% Gán lại vào các biến ngắn gọn để giữ nguyên cấu trúc công thức bên dưới
d1 = config.d1;
a2 = config.a2;
a3 = config.a3;
d4 = config.d4;
d5 = config.d5;
d6 = config.d6; 

% 2. KHAI BÁO BIẾN KHỚP (Ẩn số Symbolic)
% --------------------------------------
fprintf('Đang khởi tạo biến Symbolic...\n');
syms q1 q2 q3 q4 q5 q6 real

% --------------------------------------------------------------------
% 3. VIẾT TƯỜNG MINH TỪNG MA TRẬN CHUYỂN ĐỔI (Modified DH)
% --------------------------------------------------------------------

fprintf('Đang tính các ma trận T_rel (Relative)...\n');

% --- LINK 1: Frame {0} -> {1} ---
% alpha = 0, a = 0, d = d1, theta = q1
T_0_1 = [
    cos(q1),   -sin(q1),    0,      0;
    sin(q1),    cos(q1),    0,      0;
    0,          0,          1,      d1;
    0,          0,          0,      1
];

% --- LINK 2: Frame {1} -> {2} ---
% alpha = pi/2, a = 0, d = 0, theta = q2 + pi/2 (Offset)
th2 = q2 + sym(pi)/2; 
al1 = sym(pi)/2;      
T_1_2 = [
    cos(th2),               -sin(th2),              0,                  0;
    sin(th2)*cos(al1),      cos(th2)*cos(al1),      -sin(al1),          0; 
    sin(th2)*sin(al1),      cos(th2)*sin(al1),      cos(al1),           0;
    0,                      0,                      0,                  1
];

% --- LINK 3: Frame {2} -> {3} ---
% Link 3: a2, alpha2=0, d3=0, theta = q3
th3 = q3;
al2 = 0;
T_2_3 = [
    cos(th3),               -sin(th3),              0,                  a2;
    sin(th3)*cos(al2),      cos(th3)*cos(al2),      -sin(al2),          0; 
    sin(th3)*sin(al2),      cos(th3)*sin(al2),      cos(al2),           0;
    0,                      0,                      0,                  1
];

% --- LINK 4: Frame {3} -> {4} ---
% Link 4: a3, alpha3=0, d4, theta = q4 - pi/2
th4 = q4 - sym(pi)/2;
al3 = 0;
T_3_4 = [
    cos(th4),               -sin(th4),              0,                  a3;
    sin(th4)*cos(al3),      cos(th4)*cos(al3),      -sin(al3),          -d4*sin(al3); 
    sin(th4)*sin(al3),      cos(th4)*sin(al3),      cos(al3),           d4*cos(al3);
    0,                      0,                      0,                  1
];

% --- LINK 5: Frame {4} -> {5} ---
% Link 5: a4=0, alpha4=-pi/2, d5, theta = q5
th5 = q5;
al4 = -sym(pi)/2;
T_4_5 = [
    cos(th5),               -sin(th5),              0,                  0;
    sin(th5)*cos(al4),      cos(th5)*cos(al4),      -sin(al4),          -d5*sin(al4); 
    sin(th5)*sin(al4),      cos(th5)*sin(al4),      cos(al4),           d5*cos(al4);
    0,                      0,                      0,                  1
];

% --- LINK 6: Frame {5} -> {6} ---
% Link 6: a5=0, alpha5=pi/2, d6=0, theta = q6
th6 = q6;
al5 = sym(pi)/2;
T_5_6 = [
    cos(th6),               -sin(th6),              0,                  0;
    sin(th6)*cos(al5),      cos(th6)*cos(al5),      -sin(al5),          0; 
    sin(th6)*sin(al5),      cos(th6)*sin(al5),      cos(al5),           0;
    0,                      0,                      0,                  1
];

% --------------------------------------------------------------------
% 4. NHÂN MA TRẬN TÍCH LŨY (Forward Kinematics)
% --------------------------------------------------------------------
fprintf('Đang nhân ma trận tích lũy (T_0_i)...\n');

% T_0_1
T01 = simplify(T_0_1);

% T_0_2 = T_0_1 * T_1_2
T02 = simplify(T01 * T_1_2);

% T_0_3 = T_0_2 * T_2_3
T03 = simplify(T02 * T_2_3);

% T_0_4 = T_0_3 * T_3_4
T04 = simplify(T03 * T_3_4);

% T_0_5 = T_0_4 * T_4_5
T05 = simplify(T04 * T_4_5);

% T_0_6 = T_0_5 * T_5_6
T06 = simplify(T05 * T_5_6);

% --------------------------------------------------------------------
% 5. LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('Đang lưu kết quả vào file "ur10e_FK_symbolic.mat"...\n');

% Lưu toàn bộ workspace
save('ur10e_FK_symbolic.mat', 'T01', 'T02', 'T03', 'T04', 'T05', 'T06', 'q1','q2','q3','q4','q5','q6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Bạn có thể gõ "T06" để xem ma trận TCP cuối cùng.\n');