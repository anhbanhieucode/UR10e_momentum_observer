%% UR10e Q1 Geometry Visualization (Cực kỳ cơ bản)
clear; clc; figure('Color', 'w'); hold on; grid on; axis equal;

% 1. Giả lập thông số (Lấy d4 ngẫu nhiên hoặc theo UR10e)
d4 = 0.1639; % m (UR10e)
radius_path = 0.2; % Bán kính quỹ đạo tròn của đầu công tác
theta_path = linspace(0, 2*pi, 100);
x_center = -0.3;
y_center = 0.4
% 2. Tạo quỹ đạo mục tiêu (XYZ Target)
x_t = x_center + radius_path * cos(theta_path);
y_t = y_center + radius_path * sin(theta_path);

% 3. Khởi tạo đồ thị
h_arm = plot([0 0], [0 0], 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
h_target = plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2);
h_path = plot(x_t, y_t, 'k--', 'Color', [0.5 0.5 0.5]);
h_d4_circle = viscircles([0 0], d4, 'Color', 'g', 'LineStyle', ':'); % Vòng tròn d4

% Cố định khung hình
axis([-0.8 0.8 -0.8 0.8]);
xlabel('X (m)'); ylabel('Y (m)');
title('Mô phỏng hình học q1 với Offset d4');

% 4. Vòng lặp test q1
for i = 1:length(x_t)
    x = x_t(i);
    y = y_t(i);
    
    % --- CÔNG THỨC Q1 TÁCH RIÊNG ---
    rho = sqrt(x^2 + y^2); % Cạnh huyền tam giác vuông (Base to Target)
    
    % Tính q1 (Dựa trên hình tam giác vuông tạo bởi d4 và cạnh huyền rho)
    % phi là góc trong tam giác vuông
    phi = acos(d4 / rho); 
    theta_target = atan2(y, x);
    
    % q1 = góc hướng mục tiêu + góc lệch do d4 + 90 độ (do trục DH)
    q1 = theta_target + phi + pi/2;
    % -------------------------------
    
    % Tính toán tọa độ khớp vai (Vị trí thực của link sau khi quay q1)
    % Để kiểm tra, ta vẽ một đường thẳng từ tâm ra điểm (x,y)
    % nhưng phải "né" khoảng d4.
    x_shoulder = d4 * cos(q1 - pi/2);
    y_shoulder = d4 * sin(q1 - pi/2);
    
    % Cập nhật hình vẽ
    set(h_target, 'XData', x, 'YData', y);
    set(h_arm, 'XData', [0 x_shoulder x], 'YData', [0 y_shoulder y]);
    
    % Vẽ tam giác vuông minh họa cho step hiện tại
    h_tri = patch([0 x_shoulder x], [0 y_shoulder y], 'y', 'FaceAlpha', 0.1);
    
    pause(0.05);
    delete(h_tri); % Xóa tam giác để vẽ cái mới
end