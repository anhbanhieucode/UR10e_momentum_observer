% ur10e_verify_dynamics.m
% Kiểm tra tính chất Skew-Symmetry: (M_dot - 2C) + (M_dot - 2C)' = 0
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD DỮ LIỆU
fprintf('1. Đang load dữ liệu M và C...\n');
load('ur10e_M_symbolic.mat'); % Load M
load('ur10e_C_symbolic.mat'); % Load C

q_vars = [q1; q2; q3; q4; q5; q6];
dq_vars = [dq1; dq2; dq3; dq4; dq5; dq6];

% 2. TÍNH M_DOT (Đạo hàm toàn phần theo thời gian)
fprintf('2. Đang tính M_dot (Thời gian đạo hàm)...\n');
M_dot = sym(zeros(6,6));
for i = 1:6
    % M_dot = sum( (dM/dqi) * dqi )
    M_dot = M_dot + diff(M, q_vars(i)) * dq_vars(i);
end

% 3. TÍNH MA TRẬN KIỂM TRA N = M_dot - 2*C
fprintf('3. Đang tạo ma trận N = M_dot - 2*C...\n');
N = M_dot - 2*C;

% 4. THAY SỐ NGẪU NHIÊN ĐỂ KIỂM TRA (Tránh đơ máy do rút gọn Symbolic)
fprintf('4. Đang thay số ngẫu nhiên để kiểm tra tính chất...\n');

q_rand = rand(6, 1);
dq_rand = rand(6, 1);

% Thay số
N_num = subs(N, [q_vars; dq_vars], [q_rand; dq_rand]);
N_num = double(N_num);

% Kiểm tra N + N'
Verify_Matrix = N_num + N_num';

% 5. HIỂN THỊ KẾT QUẢ
fprintf('\n--- KẾT QUẢ KIỂM TRA ---\n');
% Giá trị này phải cực nhỏ (ví dụ 10^-14)
Max_Error = max(max(abs(Verify_Matrix)));

disp('Ma trận (N + N^T):');
disp(Verify_Matrix);

fprintf('Sai số lớn nhất: %.5e\n', Max_Error);

if Max_Error < 1e-10
    fprintf('\n===> CHÚC MỪNG: Ma trận (M_dot - 2C) là LỆCH ĐỐI XỨNG!\n');
    fprintf('Hệ thống Động lực học của bạn hoàn toàn chính xác.\n');
else
    fprintf('\n===> CẢNH BÁO: Kiểm tra thất bại. Có lỗi trong ma trận M hoặc C.\n');
end