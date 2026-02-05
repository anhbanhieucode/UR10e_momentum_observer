% ur10e_forward_kinematics_pure_symbolic.m
% Tính toán Động học thuận UR10e (Modified DH)
% Phong cách: THUẦN SYMBOLIC (Không thay số)
% Dùng để trích xuất công thức tổng quát cho báo cáo LaTeX
% --------------------------------------------------------------------
clear; clc;

% 1. KHAI BÁO BIẾN SYMBOLIC TOÀN BỘ
% ---------------------------------
fprintf('Đang khởi tạo toàn bộ biến Symbolic (q, a, d)...\n');

% Biến khớp (Joint variables)
syms q1 q2 q3 q4 q5 q6 real

% Tham số hình học (Geometric parameters)
% Lưu ý: d2, d3 = 0 và a1, a4, a5, a6 = 0 trong UR10e nên ta không khai báo thừa
syms d1 a2 a3 d4 d5 d6 real 

% --------------------------------------------------------------------
% 2. VIẾT TƯỜNG MINH TỪNG MA TRẬN CHUYỂN ĐỔI (Modified DH)
% --------------------------------------------------------------------
fprintf('Đang thiết lập ma trận T_rel...\n');

% --- LINK 1: Frame {0} -> {1} ---
% alpha = 0, a = 0, d = d1, theta = q1
T_0_1 = [
    cos(q1),   -sin(q1),    0,      0;
    sin(q1),    cos(q1),    0,      0;
    0,          0,          1,      d1;
    0,          0,          0,      1
];

% --- LINK 2: Frame {1} -> {2} ---
% alpha = pi/2, a = 0, d = 0, theta = q2 + pi/2
th2 = q2 + sym(pi)/2; 
al1 = sym(pi)/2;      
T_1_2 = [
    cos(th2),               -sin(th2),              0,                  0;
    sin(th2)*cos(al1),      cos(th2)*cos(al1),      -sin(al1),          0; 
    sin(th2)*sin(al1),      cos(th2)*sin(al1),      cos(al1),           0;
    0,                      0,                      0,                  1
];

% --- LINK 3: Frame {2} -> {3} ---
% Link 3: a2, alpha=0, d=0, theta = q3
th3 = q3;
al2 = 0;
T_2_3 = [
    cos(th3),               -sin(th3),              0,                  a2;
    sin(th3)*cos(al2),      cos(th3)*cos(al2),      -sin(al2),          0; 
    sin(th3)*sin(al2),      cos(th3)*sin(al2),      cos(al2),           0;
    0,                      0,                      0,                  1
];

% --- LINK 4: Frame {3} -> {4} ---
% Link 4: a3, alpha=0, d=d4, theta = q4 - pi/2
th4 = q4 - sym(pi)/2;
al3 = 0;
T_3_4 = [
    cos(th4),               -sin(th4),              0,                  a3;
    sin(th4)*cos(al3),      cos(th4)*cos(al3),      -sin(al3),          -d4*sin(al3); 
    sin(th4)*sin(al3),      cos(th4)*sin(al3),      cos(al3),           d4*cos(al3);
    0,                      0,                      0,                  1
];

% --- LINK 5: Frame {4} -> {5} ---
% Link 5: a=0, alpha=-pi/2, d=d5, theta = q5
th5 = q5;
al4 = -sym(pi)/2;
T_4_5 = [
    cos(th5),               -sin(th5),              0,                  0;
    sin(th5)*cos(al4),      cos(th5)*cos(al4),      -sin(al4),          -d5*sin(al4); 
    sin(th5)*sin(al4),      cos(th5)*sin(al4),      cos(al4),           d5*cos(al4);
    0,                      0,                      0,                  1
];

% --- LINK 6: Frame {5} -> {6} ---
% Link 6: a=0, alpha=pi/2, d=d6, theta = q6
th6 = q6;
al5 = sym(pi)/2;
T_5_6 = [
    cos(th6),               -sin(th6),              0,                  0;
    sin(th6)*cos(al5),      cos(th6)*cos(al5),      -sin(al5),          0; 
    sin(th6)*sin(al5),      cos(th6)*sin(al5),      cos(al5),           0;
    0,                      0,                      0,                  1
];

% --------------------------------------------------------------------
% 3. NHÂN MA TRẬN TÍCH LŨY
% --------------------------------------------------------------------
fprintf('Đang nhân ma trận tích lũy (Có thể mất chút thời gian)...\n');

% Dùng simplify để MATLAB tự rút gọn cos(q+pi/2) thành -sin(q)
T01 = simplify(T_0_1);
T02 = simplify(T01 * T_1_2);
T03 = simplify(T02 * T_2_3);
T04 = simplify(T03 * T_3_4);
T05 = simplify(T04 * T_4_5);
T06 = simplify(T05 * T_5_6);

% --------------------------------------------------------------------
% 4. LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('Đang lưu kết quả vào "ur10e_pure_symbolic.mat"...\n');
save('ur10e_pure_symbolic.mat', 'T01', 'T02', 'T03', 'T04', 'T05', 'T06', ...
     'q1','q2','q3','q4','q5','q6', ...
     'd1','a2','a3','d4','d5','d6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Gõ "T02" để xem ma trận tổng quát chứa a2, d1...\n');