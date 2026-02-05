% ur10e_calc_M_matrix.m
% Tính toán Ma trận Khối lượng M(q) tổng hợp (6x6)
% Công thức: M = sum( m * Jv' * Jv  +  Jr' * (R * I * R') * Jr )
% --------------------------------------------------------------------

clear; clc;

% 1. LOAD DỮ LIỆU TỪ CÁC BƯỚC TRƯỚC
% --------------------------------------------------------------------
fprintf('1. Đang load dữ liệu từ các file .mat và config...\n');

% Load Constants (Khối lượng, Quán tính)
if ~isfile('config.m')
    error('Thiếu file ur10e_constants.m');
end
config = config();

% Load FK (Rotation Matrices)
load('ur10e_FK_symbolic.mat'); 

% Load Jv (Translational Jacobian)
load('ur10e_Jv_symbolic.mat');

% Load Jr (Rotational Jacobian)
load('ur10e_Jr_symbolic.mat');

% 2. SẮP XẾP DỮ LIỆU VÀO CELL ARRAY (Để dùng vòng lặp cho gọn)
% --------------------------------------------------------------------
fprintf('2. Đang sắp xếp dữ liệu...\n');

% Khối lượng
m_list = [config.m1, config.m2, config.m3, config.m4, config.m5, config.m6];

% Quán tính (Local)

% Tôi sẽ sửa lại đoạn này để lấy đúng trường I trong struct config:
I_list = {config.I1, config.I2, config.I3, config.I4, config.I5, config.I6};

% Ma trận quay R (Lấy từ T01...T06)
R_list = {T01(1:3,1:3), T02(1:3,1:3), T03(1:3,1:3), ...
          T04(1:3,1:3), T05(1:3,1:3), T06(1:3,1:3)};

% Jacobian Tịnh tiến
Jv_list = {Jv1, Jv2, Jv3, Jv4, Jv5, Jv6};

% Jacobian Quay
Jr_list = {Jr1, Jr2, Jr3, Jr4, Jr5, Jr6};

% 3. TÍNH TOÁN M(q) - VÒNG LẶP CỘNG DỒN
% --------------------------------------------------------------------
fprintf('3. Đang tính toán M(q) (Quá trình này mất khoảng 30s-1p)...\n');

M = sym(zeros(6, 6)); % Khởi tạo ma trận 0

for i = 1:6
    fprintf('   -> Đang cộng dồn Link %d...\n', i);
    
    % Lấy dữ liệu của Link i
    m_i  = m_list(i);
    I_i  = I_list{i};     % Inertia Local
    R_i  = R_list{i};     % Rotation Global
    Jv_i = Jv_list{i};
    Jr_i = Jr_list{i};
    
    % --- THÀNH PHẦN TỊNH TIẾN (Translational) ---
    % M_trans = m * Jv^T * Jv
    M_trans = m_i * (transpose(Jv_i) * Jv_i);
    
    % --- THÀNH PHẦN QUAY (Rotational) ---
    % Bước 1: Chuyển I từ Local sang Global: I_global = R * I_local * R^T
    I_global = R_i * I_i * transpose(R_i);
    
    % Bước 2: Tính M_rot = Jr^T * I_global * Jr
    M_rot = transpose(Jr_i) * I_global * Jr_i;
    
    % --- CỘNG DỒN ---
    M = M + M_trans + M_rot;
end

% Đảm bảo tính đối xứng (Loại bỏ sai số nhỏ nếu có)
M = (M + transpose(M)) / 2;

% 4. LƯU KẾT QUẢ
% --------------------------------------------------------------------
fprintf('4. Đang lưu kết quả vào file "ur10e_M_symbolic.mat"...\n');

% Lưu M và các biến khớp để tiện load lại
save('ur10e_M_symbolic.mat', 'M', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6');

fprintf('=== HOÀN TẤT ===\n');
fprintf('Ma trận M(q) đã được lưu. Do biểu thức rất dài, \n');
fprintf('bạn nên dùng lệnh "load" để dùng lại thay vì hiển thị trực tiếp.\n');