function ur10e_circular_motion()
    % 1. Load parameters từ file config của bạn
    p = config();
    
    % 2. Thiết lập quỹ đạo hình tròn
    center_x = 0.6; 
    center_y = -0.5;
    z_fixed  = 0.4; 
    radius   = 0.15;
    num_steps = 50; 
    theta_arc = linspace(0, 2*pi, num_steps);
    
    % --- KHỞI TẠO MẢNG LƯU TRỮ DỮ LIỆU KHỚP ---
    q_history = zeros(num_steps, 6);
    
    % Tạo figure mô phỏng 3D
    fig_sim = figure('Color', 'w', 'Name', 'UR10e Circular Motion 3D');
    hold on; grid on; axis equal; view(0,0);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    % Lặp qua từng điểm trên quỹ đạo
    for k = 1:num_steps

        x_real = center_x + radius * cos(theta_arc(k));
        y_real = center_y + radius * sin(theta_arc(k));
        z_real = z_fixed;
        
        x = x_real + p.d5*cos(atan2(y_real,x_real));
        y = y_real + p.d5*sin(atan2(y_real,x_real));
        z = z_real + p.d6;
        
        % --- 3. Giải Động học ngược (IK) ---
        
        % Tính q1
        q1 = acos(p.d4 / sqrt(x^2 + y^2)) + atan2(y,x) + pi/2;
        
        % Các thành phần trung gian
        r_sq = x^2 + y^2 - p.d4^2;
        h = z - p.d1;
        D = (r_sq + h^2 + p.a2^2 - p.a3^2) / (2 * p.a2 * sqrt(r_sq + h^2));
        
        % Tính q2
        q2 = pi/2 - atan2(h, sqrt(r_sq)) - acos(D);
        
        % Tính q3
        D3 = (r_sq + h^2 - p.a2^2 - p.a3^2) / (2 * p.a2 * p.a3);
        q3 = acos(max(min(D3, 1), -1));
        
        % --- GIẢI CỔ TAY (WRIST IK) ---
        C1 = cos(q1); S1 = sin(q1);
        C23 = cos(q2 + q3); S23 = sin(q2 + q3);
        R03 = [ C1*C23, -C1*S23,  S1;
                S1*C23, -S1*S23, -C1;
                S23,     C23,     0];
        
        R_target = [1 0 0; 0 1 0; 0 0 -1];
        R36 = R03' * R_target;
        
        q5 = acos(max(min(R36(3,3), 1), -1)); 
        q4 = atan2(R36(2,3), R36(1,3));
        q6 = atan2(R36(3,2), -R36(3,1));
        
        q_current = [q1, q2, q3, q4, q5, q6];
        
        % --- LƯU GIÁ TRỊ VÀO LỊCH SỬ ---
        q_history(k, :) = q_current;
        
        % 4. Cập nhật Visualization
        figure(fig_sim); % Đảm bảo vẽ đúng cửa sổ 3D
        cla; 
        draw_robot_at_config(q_current, p);
        
        plot3(center_x + radius*cos(theta_arc(1:k)), ...
              center_y + radius*sin(theta_arc(1:k)), ...
              z_fixed*ones(1,k), 'k--', 'LineWidth', 1.5);
          
        title(sprintf('Step %d/%d', k, num_steps));
        drawnow; 
    end
    
    % --- VẼ ĐỒ THỊ 6 KHỚP SAU KHI CHẠY XONG ---
    plot_joint_angles(q_history, num_steps);
end

function plot_joint_angles(q_data, steps)
    % Tạo cửa sổ mới cho đồ thị
    figure('Color', 'w', 'Name', 'Joint Angles Analysis', 'Position', [100, 100, 1000, 600]);
    
    joint_names = {'q1 (Base)', 'q2 (Shoulder)', 'q3 (Elbow)', ...
                   'q4 (Wrist 1)', 'q5 (Wrist 2)', 'q6 (Wrist 3)'};
    
    t = 1:steps;
    
    for i = 1:6
        subplot(2, 3, i);
        plot(t, q_data(:, i), 'LineWidth', 2, 'Color', 'b');
        grid on;
        title(joint_names{i}, 'FontSize', 12, 'FontWeight', 'bold');
        xlabel('Step');
        ylabel('Angle (rad)');
        
        % Hiển thị giá trị max/min để dễ theo dõi biên độ
        text(steps*0.05, max(q_data(:,i)), sprintf('Max: %.2f', max(q_data(:,i))), 'Color', 'r');
        text(steps*0.05, min(q_data(:,i)), sprintf('Min: %.2f', min(q_data(:,i))), 'Color', 'k');
    end
end

function draw_robot_at_config(q, p)
    % Hàm vẽ robot (giữ nguyên logic cũ)
    dh = [
        0,      0,      p.d1, q(1);
        pi/2,   0,      0,    q(2) + pi/2;
        0,      p.a2,   0,    q(3);
        0,      p.a3,   p.d4, q(4) - pi/2;
        -pi/2,  0,      p.d5, q(5);
        pi/2,   0,      p.d6, q(6)
    ];
    
    T = eye(4);
    radii = p.radii;
    lengths = p.lengths;
    
    for i = 1:6
        alpha = dh(i,1); a = dh(i,2); d = dh(i,3); theta = dh(i,4);
        Ti = [cos(theta), -sin(theta), 0, a;
              sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
              sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
              0, 0, 0, 1];
        T = T * Ti;
        
        com_local = p.(sprintf('com%d', i));
        
        if i == 2 || i == 3, axis_sym = 'X'; else, axis_sym = 'Z'; end
        draw_com_centered_cylinder(T, com_local, axis_sym, radii(i), lengths(i));
        
        if i == 6
            s = 0.15;
            line([T(1,4) T(1,4)+T(1,3)*s], [T(2,4) T(2,4)+T(2,3)*s], [T(3,4) T(3,4)+T(3,3)*s], 'Color', 'b', 'LineWidth', 2);
        end
    end
end

function draw_com_centered_cylinder(T, com_local, axis_sym, r, L)
    % Hàm vẽ hình trụ (giữ nguyên logic cũ)
    [xc, yc, zc] = cylinder(r, 20);
    zc = (zc - 0.5) * L;
    R_align = eye(3);
    if strcmp(axis_sym, 'X')
        R_align = [0 0 1; 0 1 0; -1 0 0];
    elseif strcmp(axis_sym, 'Y')
        R_align = [1 0 0; 0 0 -1; 0 1 0];
    end
    R_total = T(1:3, 1:3) * R_align;
    com_global_pos = T(1:3, 1:4) * [com_local; 1];
    for j = 1:size(xc, 2)
        p1 = R_total * [xc(1,j); yc(1,j); zc(1,j)] + com_global_pos;
        p2 = R_total * [xc(2,j); yc(2,j); zc(2,j)] + com_global_pos;
        xc(1,j) = p1(1); xc(2,j) = p2(1);
        yc(1,j) = p1(2); yc(2,j) = p2(2);
        zc(1,j) = p1(3); zc(2,j) = p2(3);
    end
    surf(xc, yc, zc, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end