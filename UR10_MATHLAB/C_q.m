% ur10e_calc_C_fast.m
% Tính toán ma trận Coriolis C(q, dq) - Phiên bản tốc độ cao (Bỏ Simplify)
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD MA TRẬN M SYMBOLIC
if isfile('ur10e_M_symbolic.mat')
    fprintf('Đang load ma trận M từ file "ur10e_M_symbolic.mat"...\n');
    load('ur10e_M_symbolic.mat'); 
else
    error('Lỗi: Cần chạy file tính M trước!');
end

% 2. KHAI BÁO BIẾN VẬN TỐC dq
syms dq1 dq2 dq3 dq4 dq5 dq6 real
q = [q1; q2; q3; q4; q5; q6];
dq = [dq1; dq2; dq3; dq4; dq5; dq6];
n = 6;
C = sym(zeros(n, n));

fprintf('Bắt đầu tính ma trận C bằng ký hiệu Christoffel (Bỏ Simplify)...\n');

% 3. TÍNH TOÁN THEO CÔNG THỨC
tic; % Bấm giờ để theo dõi
for i = 1:n
    fprintf('   -> Đang tính hàng %d...\n', i);
    for j = 1:n
        c_ij = sym(0);
        for k = 1:n
            % Tính ký hiệu Christoffel bậc 1: Gamma_ijk
            % 0.5 * (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i)
            term1 = diff(M(i,j), q(k));
            term2 = diff(M(i,k), q(j));
            term3 = diff(M(j,k), q(i));
            
            gamma_ijk = 0.5 * (term1 + term2 - term3);
            
            % Cộng dồn trực tiếp
            c_ij = c_ij + gamma_ijk * dq(k);
        end
        % Lưu trực tiếp vào ma trận, không dùng simplify
        C(i,j) = c_ij;
    end
end
t_calc = toc;

% 4. LƯU KẾT QUẢ NGAY LẬP TỨC
fprintf('Tính toán xong sau %.2f giây. Đang lưu file...\n', t_calc);

save('ur10e_C_symbolic.mat', 'C', 'q1','q2','q3','q4','q5','q6', 'dq1','dq2','dq3','dq4','dq5','dq6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Kết quả đã được lưu vào "ur10e_C_symbolic.mat".\n');
fprintf('Lưu ý: Do không rút gọn, hãy dùng "vpa(C(i,j), 4)" để xem giá trị.\n');