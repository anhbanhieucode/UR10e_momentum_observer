function params = ur10e_constants()
    % ur10e_constants.m
    % Chứa toàn bộ thông số kỹ thuật của UR10e (Số thực)
    % Dùng để thay thế (subs) vào các công thức Symbolic.
    % ---------------------------------------------------

    % 1. THÔNG SỐ DH (Đơn vị: mét)
    % ----------------------------
    params.d1 = 0.181;
    params.a2 = 0.613;
    params.a3 = 0.572;
    params.d4 = 0.174;
    params.d5 = 0.120;
    params.d6 = 0.171;      % Thường là 0 hoặc tính vào End-Effector
    % 2. KHỐI LƯỢNG (Đơn vị: kg)
    % --------------------------
    params.m1 = 7.37;
    params.m2 = 13.05;
    params.m3 = 3.99;
    params.m4 = 2.10;
    params.m5 = 1.98;
    params.m6 = 0.62;
    
    % Khai báo lại r và L để khớp với file config của bạn (vì file đó ko lưu r, L vào struct)
    params.radii = [0.05, 0.05, 0.045, 0.04, 0.04, 0.04];
    params.lengths = [0.181, 0.613, 0.572, 0.174, 0.120, 0.04];

    % 3. TỌA ĐỘ TRỌNG TÂM COM (Local Frame) (x, y, z)
    % -----------------------------------------------
    params.com1 = [0.021; 0; 0.027];
    params.com2 = [0.38;  0; 0.158];
    params.com3 = [0.24;  0; 0.068];
    params.com4 = [0.0;   0.007; 0.018];
    params.com5 = [0.0;   0.007; 0.018];
    params.com6 = [0.0;   0.0;  -0.026];

    % 4. MA TRẬN QUÁN TÍNH TẠI TÂM (I_cm) (kg*m^2)
    % Tính toán số học dựa trên xấp xỉ hình trụ
    % --------------------------------------------
    
    % -- Link 1 (Z axis) --
    r=0.05; L=0.181; m=params.m1;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I1 = diag([I_trans, I_trans, I_long]);

    % -- Link 2 (X axis !!!) --
    r=0.06; L=0.613; m=params.m2;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I2 = diag([I_long, I_trans, I_trans]);

    % -- Link 3 (X axis !!!) --
    r=0.045; L=0.572; m=params.m3;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I3 = diag([I_long, I_trans, I_trans]);

    % -- Link 4 (Z axis) --
    r=0.04; L=0.174; m=params.m4;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I4 = diag([I_trans, I_trans, I_long]);

    % -- Link 5 (Z axis) --
    r=0.04; L=0.120; m=params.m5;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I5 = diag([I_trans, I_trans, I_long]);

    % -- Link 6 (Z axis) --
    r=0.04; L=0.04; m=params.m6;
    I_long=0.5*m*r^2; I_trans=(1/12)*m*(3*r^2+L^2);
    params.I6 = diag([I_trans, I_trans, I_long]);

    % 5. HỖ TRỢ THAY THẾ SYMBOLIC
    % Tạo 2 danh sách để dùng hàm subs(bieu_thuc, old, new)
    % -----------------------------------------------------
    
    % Danh sách tên biến Symbolic (String)
    params.sym_names = {'d1', 'a2', 'a3', 'd4', 'd5', ...
                        'm1', 'm2', 'm3', 'm4', 'm5', 'm6'};
                    
    % Danh sách giá trị tương ứng
    params.sym_vals  = [params.d1, params.a2, params.a3, params.d4, params.d5, ...
                        params.m1, params.m2, params.m3, params.m4, params.m5, params.m6];

end