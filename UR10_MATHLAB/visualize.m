function ur10e_com_centered_visualizer()
    % 1. Load parameters
    p = config();
    
    % 2. Setup configuration (Tư thế để kiểm tra)
    q = [0, pi/4, 0, 0, 0, 0]; 
    
    % DH Table [alpha_i-1, a_i-1, d_i, theta_i]
    dh = [
        0,      0,      p.d1, q(1);
        pi/2,   0,      0,    q(2) + pi/2;
        0,      p.a2,   0,    q(3);
        0,      p.a3,   p.d4, q(4) - pi/2;
        -pi/2,  0,      p.d5, q(5);
        pi/2,   0,      p.d6, q(6)
    ];

    T = eye(4);
    figure('Color', 'w', 'Name', 'UR10e COM Centered Visualization');
    hold on; grid on; axis equal; view(3);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('UR10e: Cylinders Centered at COM (Using Inertia Axis)');

    % Khai báo lại r và L để khớp với file config của bạn (vì file đó ko lưu r, L vào struct)
    
    radii = p.radii;
    lengths = p.lengths;

    for i = 1:6
        % Forward Kinematics
        alpha = dh(i,1); a = dh(i,2); d = dh(i,3); theta = dh(i,4);
        Ti = [cos(theta), -sin(theta), 0, a;
              sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha);
              sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha);
              0, 0, 0, 1];
        T = T * Ti;
        
        % Tọa độ COM toàn cục
        com_local = p.(sprintf('com%d', i));
        com_global = T * [com_local; 1];
        
        % Vẽ điểm COM
        plot3(com_global(1), com_global(2), com_global(3), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

        % Xác định trục đối xứng từ config của bạn
        if i == 2 || i == 3
            axis_sym = 'X';
        else
            axis_sym = 'Z';
        end
        
        % Vẽ hình trụ với TÂM là COM
        draw_com_centered_cylinder(T, com_local, axis_sym, radii(i), lengths(i));
        
        % Vẽ hệ trục tọa độ tại khớp để dễ nhìn
        scale = 0.1;
        quiver3(T(1,4), T(2,4), T(3,4), T(1,1)*scale, T(2,1)*scale, T(3,1)*scale, 'r');
        quiver3(T(1,4), T(2,4), T(3,4), T(1,3)*scale, T(2,3)*scale, T(3,3)*scale, 'b');
    end
    
    view(45, 30);
end

function draw_com_centered_cylinder(T, com_local, axis_sym, r, L)
    % Tạo hình trụ đơn vị
    [xc, yc, zc] = cylinder(r, 20);
    
    % Dịch chuyển zc sao cho tâm hình trụ nằm tại 0 (biến nó thành từ -L/2 đến L/2)
    zc = (zc - 0.5) * L;
    
    % Ma trận xoay để định hướng hình trụ theo trục đối xứng cục bộ
    R_align = eye(3);
    if strcmp(axis_sym, 'X')
        R_align = [0 0 1; 0 1 0; -1 0 0]; % Xoay Z sang X
    elseif strcmp(axis_sym, 'Y')
        R_align = [1 0 0; 0 0 -1; 0 1 0]; % Xoay Z sang Y
    end
    
    % Ma trận biến đổi: Quay theo khớp -> Quay theo trục đối xứng -> Dịch đến COM
    R_total = T(1:3, 1:3) * R_align;
    com_global_pos = T(1:3, 1:4) * [com_local; 1];

    % Áp dụng biến đổi cho từng điểm của mặt hình trụ
    for j = 1:size(xc, 2)
        p1 = R_total * [xc(1,j); yc(1,j); zc(1,j)] + com_global_pos;
        p2 = R_total * [xc(2,j); yc(2,j); zc(2,j)] + com_global_pos;
        xc(1,j) = p1(1); xc(2,j) = p2(1);
        yc(1,j) = p1(2); yc(2,j) = p2(2);
        zc(1,j) = p1(3); zc(2,j) = p2(3);
    end
    
    surf(xc, yc, zc, 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end