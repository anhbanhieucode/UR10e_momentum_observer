% ur10e_calc_inertia_matrix.m
% Tính toán Ma trận quán tính tại tâm (I_cm) cho UR10e
% Phương pháp: Xấp xỉ hình trụ (Cylinder)
% Quy ước trục:
% - Link 2, 3: Trục đối xứng trùng trục X của DH
% - Link 1, 4, 5, 6: Trục đối xứng trùng trục Z của DH
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD THÔNG SỐ TỪ FILE CẤU HÌNH
% (Đảm bảo bạn đã có file ur10e_constants.m hoặc config.m)
if isfile('config.m')
    config = config();
else
    error('Lỗi: Không tìm thấy file cấu hình (ur10e_constants.m hoặc config.m)');
end

fprintf('Đang tính toán lại ma trận quán tính...\n');

% --------------------------------------------------------------------
% LINK 1 (Shoulder)
% Trục đối xứng: Z (Vertical)
% --------------------------------------------------------------------
m = config.m1; 
r = 0.05;       % Bán kính ước lượng (m)
L = config.d1;  % Chiều dài (0.181)

I_long  = 0.5 * m * r^2;               % Quanh trục Z
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục X, Y

% Sắp xếp: [Lớn, Lớn, Nhỏ]
I1 = diag([I_trans, I_trans, I_long]);

fprintf('Link 1 (Axis Z): I_zz=%.5f, I_xx=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LINK 2 (Upper Arm) -> QUAN TRỌNG
% Trục đối xứng: X (Horizontal theo DH)
% --------------------------------------------------------------------
m = config.m2; 
r = 0.05; 
L = config.a2;  % Chiều dài (0.613)

I_long  = 0.5 * m * r^2;               % Quanh trục X (Cuộn)
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục Y, Z (Gật/Xoay)

% Sắp xếp: [Nhỏ, Lớn, Lớn]
I2 = diag([I_long, I_trans, I_trans]);
%I2 = diag([I_trans, I_trans, I_long]);

fprintf('Link 2 (Axis X): I_xx=%.5f, I_yy=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LINK 3 (Forearm) -> QUAN TRỌNG
% Trục đối xứng: X (Horizontal theo DH)
% --------------------------------------------------------------------
m = config.m3; 
r = 0.045; 
L = config.a3;  % Chiều dài (0.572)

I_long  = 0.5 * m * r^2;               % Quanh trục X
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục Y, Z

% Sắp xếp: [Nhỏ, Lớn, Lớn]
I3 = diag([I_long, I_trans, I_trans]);
%I3 = diag([I_trans, I_trans, I_long]);

fprintf('Link 3 (Axis X): I_xx=%.5f, I_yy=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LINK 4 (Wrist 1)
% Trục đối xứng: Z
% --------------------------------------------------------------------
m = config.m4; 
r = 0.04; 
L = config.d4; % Chiều dài (0.174)

I_long  = 0.5 * m * r^2;               % Quanh trục Z
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục X, Y

% Sắp xếp: [Lớn, Lớn, Nhỏ]
I4 = diag([I_trans, I_trans, I_long]);

fprintf('Link 4 (Axis Z): I_zz=%.5f, I_xx=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LINK 5 (Wrist 2)
% Trục đối xứng: Z
% --------------------------------------------------------------------
m = config.m5; 
r = 0.04; 
L = config.d5; % Chiều dài (0.120)

I_long  = 0.5 * m * r^2;               % Quanh trục Z
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục X, Y

% Sắp xếp: [Lớn, Lớn, Nhỏ]
I5 = diag([I_trans, I_trans, I_long]);

fprintf('Link 5 (Axis Z): I_zz=%.5f, I_xx=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LINK 6 (Wrist 3)
% Trục đối xứng: Z
% --------------------------------------------------------------------
m = config.m6; 
r = 0.04; 
L = 0.04;      % Rất ngắn

I_long  = 0.5 * m * r^2;               % Quanh trục Z
I_trans = (1/12) * m * (3*r^2 + L^2);  % Quanh trục X, Y

% Sắp xếp: [Lớn, Lớn, Nhỏ]
I6 = diag([I_trans, I_trans, I_long]);

fprintf('Link 6 (Axis Z): I_zz=%.5f, I_xx=%.5f\n', I_long, I_trans);


% --------------------------------------------------------------------
% LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('Đang lưu kết quả vào file "ur10e_inertia.mat"...\n');

save('ur10e_inertia.mat', 'I1', 'I2', 'I3', 'I4', 'I5', 'I6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Bạn có thể load file này để dùng cho bước tính M(q).\n');