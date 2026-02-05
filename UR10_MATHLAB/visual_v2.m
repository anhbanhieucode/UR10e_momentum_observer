% ur10e_visualize_from_fk.m
% Visualize Robot dựa trên Ma trận FK Symbolic đã lưu
% Mục đích: Kiểm tra xem công thức FK có khớp với hình học vật lý không.
% --------------------------------------------------------------------
clear; clc; close all;

% 1. LOAD DỮ LIỆU
% ---------------
fprintf('1. Đang load dữ liệu cấu hình và FK...\n');

if ~isfile('config.m') || ~isfile('ur10e_FK_symbolic.mat')
    error('Thiếu file "config.m" hoặc "ur10e_FK_symbolic.mat". Hãy chạy các bước trước đó!');
end

p = config(); % Load thông số hình học (radii, lengths, com)
load('ur10e_FK_symbolic.mat'); % Load T01...T06 và biến symbolic q1...q6

% 2. CẤU HÌNH GÓC KHỚP KIỂM TRA (Thay đổi số ở đây để test)
% ---------------------------------------------------------
q_test = [0, pi/2, 0, 0, 0, 0]; % Dựng thẳng đứng (Zero pos)
% q_test = [0, -pi/2, 0, -pi/2, 0, 0]; % Tư thế Home truyền thống
% q_test = [0, -pi/4, pi/4, -pi/2, -pi/2, 0]; % Tư thế cong tự nhiên

fprintf('2. Góc khớp kiểm tra: [%s]\n', num2str(q_test));

% 3. THAY SỐ VÀO BIỂU THỨC SYMBOLIC (SUBS)
% ----------------------------------------
fprintf('3. Đang tính toán Forward Kinematics (Xin chờ vài giây...)\n');

% Gom biến và giá trị để thay thế 1 lần (nhanh hơn)
sym_vars = {q1, q2, q3, q4, q5, q6};
val_vars = num2cell(q_test);

% Chuyển đổi từ Symbolic sang Double (Số thực)
% Lưu ý: T01...T06 là biến được load từ file .mat
T_num_1 = double(subs(T01, sym_vars, val_vars));
T_num_2 = double(subs(T02, sym_vars, val_vars));
T_num_3 = double(subs(T03, sym_vars, val_vars));
T_num_4 = double(subs(T04, sym_vars, val_vars));
T_num_5 = double(subs(T05, sym_vars, val_vars));
T_num_6 = double(subs(T06, sym_vars, val_vars));

% Gom vào một cell array để dễ loop
T_list = {T_num_1, T_num_2, T_num_3, T_num_4, T_num_5, T_num_6};

% 4. VẼ ROBOT
% -----------
fprintf('4. Đang vẽ Robot...\n');
figure('Color', 'w', 'Name', 'UR10e Symbolic FK Visualization');
hold on; grid on; axis equal; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(['UR10e Pose at q = [' num2str(q_test, '%.2f ') ']']);

% --- Vẽ Đế (Base) tĩnh ---
draw_frame(eye(4), 0.15); % Gốc tọa độ thế giới
plot3(0,0,0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% --- Vẽ các Link ---
for i = 1:6
    % Lấy ma trận Transformation của Link i (Từ Symbolic tính ra)
    T_current = T_list{i};
    
    % Lấy thông số Vật lý (Từ Config)
    com_local = p.(sprintf('com%d', i));
    radius = p.radii(i);
    len = p.lengths(i);
    
    % --- LOGIC TRỤC ĐỐI XỨNG (QUAN TRỌNG) ---
    % Phải khớp với cách bạn tính Inertia
    if i == 2 || i == 3
        axis_sym = 'X'; % Link 2, 3 nằm ngang (theo DH Modified của bạn)
    else
        axis_sym = 'Z'; % Các Link còn lại dựng đứng
    end
    
    % 1. Vẽ Hệ trục tọa độ gắn trên Link (Frame {i})
    draw_frame(T_current, 0.1);
    
    % 2. Vẽ Điểm Trọng Tâm (COM)
    % Chuyển COM từ Local sang Global bằng chính ma trận T của FK
    com_global = T_current * [com_local; 1];
    plot3(com_global(1), com_global(2), com_global(3), ...
        'o', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k');
    
    % 3. Vẽ Hình Trụ đại diện cho Link (Tâm tại COM)
    draw_com_centered_cylinder(T_current, com_local, axis_sym, radius, len, i);
    
    % Vẽ đường nối ảo từ khớp trước đến khớp này để dễ nhìn (Optional)
    if i == 1
        prev_pos = [0;0;0];
    else
        prev_T = T_list{i-1};
        prev_pos = prev_T(1:3, 4);
    end
    curr_pos = T_current(1:3, 4);
    plot3([prev_pos(1), curr_pos(1)], [prev_pos(2), curr_pos(2)], [prev_pos(3), curr_pos(3)], ...
        'k--', 'LineWidth', 0.5);
end

% Set view đẹp
view(45, 30);
xlim([-1 1]); ylim([-1 1]); zlim([-0.2 1.2]);
fprintf('=== HOÀN TẤT ===\n');


% =========================================================================
% CÁC HÀM HỖ TRỢ VẼ
% =========================================================================

function draw_frame(T, scale)
    % Vẽ các mũi tên RGB biểu diễn hệ trục tọa độ
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    quiver3(p(1), p(2), p(3), R(1,1), R(2,1), R(3,1), scale, 'r', 'LineWidth', 1.5); % X
    quiver3(p(1), p(2), p(3), R(1,2), R(2,2), R(3,2), scale, 'g', 'LineWidth', 1.5); % Y
    quiver3(p(1), p(2), p(3), R(1,3), R(2,3), R(3,3), scale, 'b', 'LineWidth', 1.5); % Z
end

function draw_com_centered_cylinder(T, com_local, axis_sym, r, L, link_idx)
    % Tạo lưới hình trụ chuẩn (dọc trục Z, từ 0 đến 1)
    [xc, yc, zc] = cylinder(r, 20);
    
    % 1. Scale chiều dài và Dịch chuyển tâm về gốc (0,0,0)
    % Biến thành hình trụ từ -L/2 đến +L/2
    zc = (zc - 0.5) * L;
    
    % 2. Xoay hình trụ cho đúng trục đối xứng (Local Rotation)
    % Mặc định cylinder() tạo dọc Z. 
    R_align = eye(3);
    if strcmp(axis_sym, 'X')
        % Xoay 90 độ quanh Y: Z -> X
        R_align = [0 0 1; 0 1 0; -1 0 0]; 
    elseif strcmp(axis_sym, 'Y')
        % Xoay 90 độ quanh X: Z -> -Y (hoặc tương tự)
        R_align = [1 0 0; 0 0 -1; 0 1 0];
    end
    
    % 3. Áp dụng Ma trận biến đổi
    % World_Pos = T_link * (R_align * Vertex_Pos + COM_offset)
    % Lưu ý: Hình trụ được vẽ với tâm tại COM của Link
    
    % Tính toán vị trí COM trong World frame (để tịnh tiến hình trụ đến đó)
    % Logic: T là pose của Frame khớp. COM nằm lệch so với Frame khớp một đoạn com_local.
    % Hình trụ được vẽ bao quanh COM.
    
    % Bước A: Xoay vertices hình trụ theo trục đối xứng
    % Bước B: Dịch vertices đi một đoạn bằng vector COM (trong hệ Local)
    % Bước C: Nhân với T để ra World
    
    % Ma trận quay tổng hợp: R_khớp * R_align
    R_total = T(1:3, 1:3) * R_align;
    
    % Vị trí tâm hình trụ (chính là COM trong World Frame)
    com_world = T(1:3, 1:4) * [com_local; 1];
    
    % Biến đổi từng điểm lưới
    for k = 1:size(xc, 2)
        % Véc tơ từ tâm hình trụ (0,0,0) ra mặt trụ
        vec_surf = [xc(1,k); yc(1,k); zc(1,k)];
        vec_surf_2 = [xc(2,k); yc(2,k); zc(2,k)];
        
        % Xoay vector này theo hướng trục đối xứng và hướng của Link
        pt1 = R_total * vec_surf + com_world;
        pt2 = R_total * vec_surf_2 + com_world;
        
        xc(1,k) = pt1(1); xc(2,k) = pt2(1);
        yc(1,k) = pt1(2); yc(2,k) = pt2(2);
        zc(1,k) = pt1(3); zc(2,k) = pt2(3);
    end
    
    % Màu sắc khác nhau cho dễ nhìn
    colors = lines(6);
    surf(xc, yc, zc, 'FaceColor', colors(link_idx,:), 'EdgeColor', 'none', 'FaceAlpha', 0.6);
end