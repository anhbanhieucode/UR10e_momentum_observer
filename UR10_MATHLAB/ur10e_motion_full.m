function ur10e_circular_motion()
    % 1. Load parameters từ file config của bạn
    p = config();
    
    % 2. Thiết lập quỹ đạo hình tròn
    center_x = -0.6; 
    center_y = -0.5;
    z_fixed  = 0.4; 
    radius   = 0.15;
    num_steps = 60; 
    theta_arc = linspace(0, 2*pi, num_steps);
    
    fig = figure('Color', 'w', 'Name', 'UR10e Circular Motion with Orientation');
    hold on; grid on; axis equal; view(3);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    % Cố định axis để quan sát quỹ đạo không bị nhảy
    axis([center_x-0.5 center_x+0.5 center_y-0.5 center_y+0.5 0 1.2]);
    
    for k = 1:num_steps
        x = center_x + radius * cos(theta_arc(k));
        y = center_y + radius * sin(theta_arc(k));
        z = z_fixed;
        
        % --- 3. Giải Động học ngược (IK) ---
        
        % q1, q2, q3 (Phần cánh tay)
        rho = sqrt(x^2 + y^2);
        q1 = acos(p.d4 / rho) + atan2(y,x) + pi/2;
        
        r_sq = x^2 + y^2 - p.d4^2;
        h = z - p.d1;
        D = (r_sq + h^2 + p.a2^2 - p.a3^2) / (2 * p.a2 * sqrt(r_sq + h^2));
        q2 = pi/2 - atan2(h, sqrt(r_sq)) - acos(max(min(D,1),-1));
        
        D3 = (r_sq + h^2 - p.a2^2 - p.a3^2) / (2 * p.a2 * p.a3);
        q3 = acos(max(min(D3, 1), -1));
        
        % --- CÔNG THỨC MỚI CỦA BẠN ---
        % q5 = arctan(-tan(q1)^2)
        q5 = atan(-(tan(q1)^2));
        
        % q4 = arcsin(-1/sin(q5)) - q2 - q3
        % Lưu ý: -1/sin(q5) rất dễ vượt quá vùng [-1, 1], ta dùng real để tránh lỗi
        val_asin = -1 / sin(q5);
        q4 = asin(max(min(val_asin, 1), -1)) - q2 - q3;
        
        q6 = 0;
        
        q_current = [q1, q2, q3, q4, q5, q6];
        
        % 4. Cập nhật Visualization
        cla; 
        draw_robot_at_config(q_current, p);
        
        % Vẽ quỹ đạo
        plot3(center_x + radius*cos(theta_arc(1:k)), ...
              center_y + radius*sin(theta_arc(1:k)), ...
              z_fixed*ones(1,k), 'k--', 'LineWidth', 1.5);
          
        title(sprintf('Step %d | q4:%.2f q5:%.2f', k, q4, q5));
        drawnow; 
    end
end

function draw_robot_at_config(q, p)
    % DH Table chuẩn cho UR10e dựa trên code của bạn
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
        com_global = T * [com_local; 1];
        
        plot3(com_global(1), com_global(2), com_global(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
        
        if i == 2 || i == 3, axis_sym = 'X'; else, axis_sym = 'Z'; end
        draw_com_centered_cylinder(T, com_local, axis_sym, radii(i), lengths(i));
        
        % Vẽ thêm hệ trục tọa độ tại End-effector (khớp 6) để kiểm tra "chúi mũi"
        if i == 6
            s = 0.15;
            line([T(1,4) T(1,4)+T(1,3)*s], [T(2,4) T(2,4)+T(2,3)*s], [T(3,4) T(3,4)+T(3,3)*s], 'Color', 'b', 'LineWidth', 2); % Trục Z
        end
    end
end

function draw_com_centered_cylinder(T, com_local, axis_sym, r, L)
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