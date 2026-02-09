% ur10e_calc_Jv_manual.m
% Tính toán tọa độ COM toàn cục và Ma trận Jacobi Tịnh tiến (Jv)
% Phong cách: Tường minh, đơn giản, dễ kiểm soát.
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD KẾT QUẢ TỪ BƯỚC TRƯỚC (T01...T06 và biến q1...q6)
if isfile('ur10e_FK_symbolic.mat')
    fprintf('Đang load ma trận T từ file "ur10e_FK_symbolic.mat"...\n');
    load('ur10e_FK_symbolic.mat');
else
    error('Lỗi: Không tìm thấy file ur10e_FK_symbolic.mat!');
end

% Tạo vector biến q để dùng cho hàm jacobian
q = [q1; q2; q3; q4; q5; q6];

config = config(); % Gọi hàm config

% --------------------------------------------------------------------
% 2. ĐỊNH NGHĨA TỌA ĐỘ COM CỤC BỘ (Local Frame)
% Dữ liệu từ bảng thông số (x, y, z, 1)
% --------------------------------------------------------------------

fprintf('Đang định nghĩa tọa độ COM cục bộ...\n');

com1_local = [config.com1;1];
com2_local = [config.com2;1];
com3_local = [config.com3;1];
com4_local = [config.com4;1];
com5_local = [config.com5;1];
com6_local = [config.com6;1];


% --------------------------------------------------------------------
% 3. TÍNH TỌA ĐỘ TOÀN CỤC (Global Position)
% P_global = T_0_i * P_local
% --------------------------------------------------------------------

fprintf('Đang tính tọa độ COM toàn cục (P_global)...\n');

% Link 1
P_global_1_hom = T01 * com1_local;
P_global_1 = simplify(P_global_1_hom(1:3)); % Chỉ lấy x,y,z

% Link 2
P_global_2_hom = T02 * com2_local;
P_global_2 = simplify(P_global_2_hom(1:3));

% Link 3
P_global_3_hom = T03 * com3_local;
P_global_3 = simplify(P_global_3_hom(1:3));

% Link 4
P_global_4_hom = T04 * com4_local;
P_global_4 = simplify(P_global_4_hom(1:3));

% Link 5
P_global_5_hom = T05 * com5_local;
P_global_5 = simplify(P_global_5_hom(1:3));

% Link 6
P_global_6_hom = T06 * com6_local;
P_global_6 = simplify(P_global_6_hom(1:3));

% --------------------------------------------------------------------
% 4. TÍNH JACOBIAN TỊNH TIẾN (Jv) BẰNG ĐẠO HÀM TỪNG PHẦN
% Jv = d(P_global) / dq
% --------------------------------------------------------------------

fprintf('Đang tính đạo hàm riêng để ra Jacobian (Jv)...\n');

fprintf('   - Tính Jv1...\n');
Jv1 = jacobian(P_global_1, q); 
% Mẹo: simplify giúp công thức gọn hơn, nhưng nếu chạy lâu quá có thể bỏ qua
Jv1 = simplify(Jv1); 

fprintf('   - Tính Jv2...\n');
Jv2 = jacobian(P_global_2, q);
Jv2 = simplify(Jv2);

fprintf('   - Tính Jv3...\n');
Jv3 = jacobian(P_global_3, q);
Jv3 = simplify(Jv3);

fprintf('   - Tính Jv4...\n');
Jv4 = jacobian(P_global_4, q);
Jv4 = simplify(Jv4);

fprintf('   - Tính Jv5...\n');
Jv5 = jacobian(P_global_5, q);
Jv5 = simplify(Jv5);

fprintf('   - Tính Jv6...\n');
Jv6 = jacobian(P_global_6, q);
Jv6 = simplify(Jv6);

% --------------------------------------------------------------------
% 5. LƯU KẾT QUẢ
% --------------------------------------------------------------------

fprintf('Đang lưu kết quả vào file "ur10e_Jv_symbolic.mat"...\n');

save('ur10e_Jv_symbolic.mat', ...
    'Jv1', 'Jv2', 'Jv3', 'Jv4', 'Jv5', 'Jv6', ...
    'P_global_1', 'P_global_2', 'P_global_3', 'P_global_4', 'P_global_5', 'P_global_6', ...
    'q1', 'q2', 'q3', 'q4', 'q5', 'q6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Bạn có thể xem công thức bằng cách gõ tên biến, ví dụ:\n');
fprintf('>> Jv3\n');
fprintf('>> P_global_6\n');