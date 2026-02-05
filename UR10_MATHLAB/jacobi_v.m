% ur10e_calc_Jr_Rdot_method.m
% Tính toán Jacobian Quay (Jr) bằng phương pháp R_dot * R_transpose
% Quy trình:
% 1. Tính R_dot (Đạo hàm toàn phần theo thời gian)
% 2. Tính Omega_skew = R_dot * R'
% 3. Rút Omega vector
% 4. Đạo hàm Omega theo dq để ra Jacobian
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD KẾT QUẢ FK (T01...T06)
if isfile('ur10e_FK_symbolic.mat')
    fprintf('Đang load ma trận T từ file "ur10e_FK_symbolic.mat"...\n');
    load('ur10e_FK_symbolic.mat');
else
    error('Lỗi: Hãy chạy file FK trước!');
end

% 2. KHAI BÁO BIẾN VẬN TỐC KHỚP (dq)
fprintf('Đang khởi tạo biến vận tốc khớp (dq)...\n');
syms dq1 dq2 dq3 dq4 dq5 dq6 real

% Gom biến để dễ xử lý
q_vars = [q1; q2; q3; q4; q5; q6];      % Biến vị trí
dq_vars = [dq1; dq2; dq3; dq4; dq5; dq6]; % Biến vận tốc

% Trích xuất các ma trận quay R từ T (3x3)
R_all = {T01(1:3,1:3), T02(1:3,1:3), T03(1:3,1:3), ...
         T04(1:3,1:3), T05(1:3,1:3), T06(1:3,1:3)};

% --------------------------------------------------------------------
% 3. VÒNG LẶP TÍNH TOÁN CHO TỪNG LINK
% --------------------------------------------------------------------

fprintf('Bắt đầu tính toán Jacobian Quay theo phương pháp R_dot...\n');

% Khởi tạo biến lưu kết quả
Jr_cell = cell(1, 6);

for i = 1:6
    fprintf('   -> Đang xử lý Link %d...\n', i);
    
    R = R_all{i};
    
    % --- BƯỚC 1: TÍNH R_DOT (Đạo hàm toàn phần) ---
    % R_dot = sum( dR/dq_k * dq_k )
    fprintf('      + Tính R_dot (Total Time Derivative)...\n');
    R_dot = sym(zeros(3,3));
    
    % Chỉ cần chạy k từ 1 đến i (vì Link i chỉ phụ thuộc q1...qi)
    for k = 1:i
        % Đạo hàm riêng của R theo q_k nhân với vận tốc dq_k
        R_dot = R_dot + diff(R, q_vars(k)) * dq_vars(k);
    end
    
    % --- BƯỚC 2: TÍNH OMEGA SKEW ---
    % Omega_skew = R_dot * R_transpose
    fprintf('      + Tính Omega_skew = R_dot * R_T...\n');
    Omega_skew = R_dot * transpose(R);
    %Omega_skew = simplify(Omega_skew); % Có thể bỏ qua để chạy nhanh hơn
    % Code kiểm tra ngay trong vòng lặp hoặc sau khi tính
    Check_Matrix = simplify(Omega_skew + transpose(Omega_skew));
    
    % Nếu kết quả là ma trận toàn số 0, thì Omega_skew ĐÚNG.
    disp('Kiểm tra Skew-Symmetric (Phải toàn là 0):');
    disp(Check_Matrix);
    % --- BƯỚC 3: TRÍCH XUẤT VECTOR OMEGA ---
    % Ma trận lệch đối xứng:
    % [ 0  -wz  wy ]
    % [ wz  0  -wx ]
    % [-wy  wx  0  ]
    wx = Omega_skew(3, 2);
    wy = Omega_skew(1, 3);
    wz = Omega_skew(2, 1);
    
    omega_vec = [wx; wy; wz];
    
    % --- BƯỚC 4: TÍNH JACOBIAN (Đạo hàm omega theo dq) ---
    % Jr = d(omega)/d(dq)
    fprintf('      + Tính Jacobian = d(omega)/d(dq)...\n');
    Jr_i = jacobian(omega_vec, dq_vars);
    
    % Rút gọn kết quả cuối cùng
    Jr_cell{i} = simplify(Jr_i);
end

% Gán ra các biến rời để lưu file
Jr1 = Jr_cell{1};
Jr2 = Jr_cell{2};
Jr3 = Jr_cell{3};
Jr4 = Jr_cell{4};
Jr5 = Jr_cell{5};
Jr6 = Jr_cell{6};

% --------------------------------------------------------------------
% 4. LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('Đang lưu kết quả vào file "ur10e_Jr_symbolic.mat"...\n');

save('ur10e_Jr_symbolic.mat', ...
    'Jr1', 'Jr2', 'Jr3', 'Jr4', 'Jr5', 'Jr6', ...
    'q1', 'q2', 'q3', 'q4', 'q5', 'q6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Bạn có thể kiểm tra kết quả bằng lệnh:\n');
fprintf('>> Jr3\n');