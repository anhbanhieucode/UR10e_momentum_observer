% ur10e_check_M_values.m
% Thay số vào ma trận Symbolic M(q) để kiểm tra tính đúng đắn.
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD DATA
if isfile('ur10e_M_symbolic.mat')
    fprintf('Đang load file M symbolic...\n');
    load('ur10e_M_symbolic.mat');
else
    error('Chưa có file ur10e_M_symbolic.mat');
end

% 2. CHỌN CẤU HÌNH TEST (Góc khớp)
% Bạn có thể sửa các số này để test các tư thế khác nhau
% Ví dụ: Tư thế "Zero" (Cánh tay duỗi thẳng/gập tùy định nghĩa zero)
q_test = [0, -pi/2, 0, -pi/2, 0, 0]; 

fprintf('---------------------------------------------------\n');
fprintf('Cấu hình kiểm tra (rad): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
    q_test(1), q_test(2), q_test(3), q_test(4), q_test(5), q_test(6));

% 3. THAY SỐ (SUBSTITUTION)
fprintf('Đang thay số vào công thức (việc này có thể mất vài giây)...\n');

% Dùng lệnh subs để thay thế q1..q6 bằng giá trị số
% Lưu ý: q1...q6 là biến symbolic đã được load từ file .mat
M_num = subs(M, ...
             {q1, q2, q3, q4, q5, q6}, ...
             {q_test(1), q_test(2), q_test(3), q_test(4), q_test(5), q_test(6)});

% Chuyển từ dạng Symbolic số sang dạng Double (Số thực) để tính toán
M_num = double(M_num);

% 4. HIỂN THỊ KẾT QUẢ
fprintf('\n--- KẾT QUẢ MA TRẬN M (6x6) ---\n');
disp(M_num);

fprintf('\n--- KIỂM TRA 1 SỐ CỤ THỂ (VÍ DỤ M11) ---\n');
fprintf('Giá trị M(1,1) = %.4f (kg*m^2)\n', M_num(1,1));
fprintf('Ý nghĩa: Quán tính tổng cộng mà mô-tơ 1 phải chịu tại tư thế này.\n');

% 5. KIỂM TRA TÍNH CHẤT VẬT LÝ (QUAN TRỌNG)
fprintf('\n--- KIỂM TRA TÍNH CHẤT VẬT LÝ ---\n');

% A. Kiểm tra đối xứng (Symmetry)
% Hiệu số giữa M và M chuyển vị phải xấp xỉ 0
sym_error = norm(M_num - M_num');
fprintf('1. Sai số đối xứng (phải ~ 0): %.5e\n', sym_error);
if sym_error < 1e-10
    fprintf('   => OK: Ma trận đối xứng.\n');
else
    fprintf('   => CẢNH BÁO: Ma trận không đối xứng!\n');
end

% B. Kiểm tra xác định dương (Positive Definite)
% Tất cả trị riêng (Eigenvalues) phải dương
eigs_val = eig(M_num);
fprintf('2. Các trị riêng (Eigenvalues): \n');
disp(eigs_val');

if all(eigs_val > 0)
    fprintf('   => OK: Ma trận xác định dương (Vật lý đúng).\n');
else
    fprintf('   => LỖI NGHIÊM TRỌNG: Có trị riêng âm hoặc bằng 0!\n');
    fprintf('      Robot không thể có khối lượng âm/vô lý.\n');
end