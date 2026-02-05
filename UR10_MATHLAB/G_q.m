% ur10e_calc_G_jacobian.m
% Tính toán Vector G(q) bằng phương pháp Jacobian Transpose
% Công thức: G = - sum( Jv_i' * m_i * g_vect )
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD DỮ LIỆU
% --------------------------------------------------------------------
fprintf('1. Đang load dữ liệu (Constants & Jacobian Tịnh tiến)...\n');

if ~isfile('config.m') || ~isfile('ur10e_Jv_symbolic.mat')
    error('Thiếu file config.m hoặc ur10e_Jv_symbolic.mat');
end

config = config();
load('ur10e_Jv_symbolic.mat'); % Chứa Jv1...Jv6

% Tạo list cho dễ lặp
m_list = [config.m1, config.m2, config.m3, config.m4, config.m5, config.m6];
Jv_list = {Jv1, Jv2, Jv3, Jv4, Jv5, Jv6};

% 2. KHAI BÁO VECTOR GIA TỐC TRỌNG TRƯỜNG
% --------------------------------------------------------------------
% Trọng lực hướng xuống theo trục Z0
g_vect = [0;0; -9.81]; 

fprintf('2. Vector trọng trường: g = [0; 0; -9.81]\n');

% 3. TÍNH TOÁN G(q) - CỘNG DỒN TỪNG LINK
% --------------------------------------------------------------------
fprintf('3. Đang tính tổng Torque trọng trường (G matrix)...\n');

G = sym(zeros(6, 1)); % Vector cột 6x1

for i = 1:6
    % Lấy dữ liệu Link i
    m_i  = m_list(i);
    Jv_i = Jv_list{i};
    
    % Tính lực trọng trường tác dụng lên Link i (F = m * g)
    F_gravity = m_i * g_vect;
    
    % Chiếu lực này về không gian khớp thông qua Jacobian Transpose
    % Tau_i = - Jv^T * F_gravity (Dấu trừ để ra lực giữ/chống lại)
    Tau_i = - transpose(Jv_i) * F_gravity;
    
    % Cộng dồn vào G tổng
    G = G + Tau_i;
    
    fprintf('   -> Đã cộng dồn Link %d\n', i);
end

% Rút gọn kết quả
fprintf('4. Đang rút gọn biểu thức (simplify)...\n');
G = simplify(G);

% 4. LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('5. Đang lưu kết quả vào "ur10e_G_symbolic.mat"...\n');

save('ur10e_G_symbolic.mat', 'G', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6');

fprintf('=== HOÀN TẤT ===\n');

% --------------------------------------------------------------------
% KIỂM TRA NHANH KẾT QUẢ (Check Vật lý)
% --------------------------------------------------------------------
fprintf('\n--- KIỂM TRA NHANH ---\n');
% Test tại vị trí robot duỗi thẳng nằm ngang (Zero Position của DH Modified)
% Tại đây, Link 2 (Vai) chịu lực lớn nhất.
q_test = [0, pi/2, 0, 0, 0, 0]; 

G_val = subs(G, {q1,q2,q3,q4,q5,q6}, q_test);
G_val = double(G_val);

fprintf('Vector G tại q=[0,0,0,0,0,0] (N.m):\n');
disp(G_val);

fprintf('Giải thích: \n');
fprintf(' - G(2) phải rất lớn (khoảng >100 Nm) vì gánh cả cánh tay đòn.\n');
fprintf(' - G(1) thường bằng 0 vì trọng lực song song trục quay đế (Z0).\n');