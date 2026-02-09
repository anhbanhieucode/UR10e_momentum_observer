% --- MATLAB SCRIPT TO EXPORT DYNAMICS TO PYTHON ---
clc; clear;

% 1. Load các file .mat của bạn (Đảm bảo tên biến bên trong khớp nhé)
% Giả sử trong file mat, tên biến lần lượt là M, C, G
load('ur10e_M_symbolic.mat'); 
load('ur10e_C_symbolic.mat');
load('ur10e_G_symbolic.mat');

% Nếu tên biến trong file mat khác (ví dụ M_sym), hãy gán lại:
% M = M_sym; 

filename = 'ur10e_dynamics.py';
fid = fopen(filename, 'w');

% Viết Header cho file Python
fprintf(fid, 'import numpy as np\n\n');
fprintf(fid, 'def get_dynamics(q, dq):\n');
fprintf(fid, '    # Unpack joints\n');
fprintf(fid, '    q1, q2, q3, q4, q5, q6 = q\n');
fprintf(fid, '    dq1, dq2, dq3, dq4, dq5, dq6 = dq\n\n');
fprintf(fid, '    # Precompute sin/cos to save speed\n');
fprintf(fid, '    s1, c1 = np.sin(q1), np.cos(q1)\n');
fprintf(fid, '    s2, c2 = np.sin(q2), np.cos(q2)\n');
fprintf(fid, '    s3, c3 = np.sin(q3), np.cos(q3)\n');
fprintf(fid, '    s4, c4 = np.sin(q4), np.cos(q4)\n');
fprintf(fid, '    s5, c5 = np.sin(q5), np.cos(q5)\n');
fprintf(fid, '    s6, c6 = np.sin(q6), np.cos(q6)\n\n');

% --- HÀM CONVERT ---
function str_out = py_format(sym_expr)
    str_out = char(sym_expr);
    % Thay thế mũ ^ thành **
    str_out = strrep(str_out, '^', '**');
    % Thay thế sin/cos để dùng biến precompute (cho gọn và nhanh)
    % Lưu ý: thay thế từ dài đến ngắn để tránh lỗi substring
    str_out = strrep(str_out, 'sin(q1)', 's1'); str_out = strrep(str_out, 'cos(q1)', 'c1');
    str_out = strrep(str_out, 'sin(q2)', 's2'); str_out = strrep(str_out, 'cos(q2)', 'c2');
    str_out = strrep(str_out, 'sin(q3)', 's3'); str_out = strrep(str_out, 'cos(q3)', 'c3');
    str_out = strrep(str_out, 'sin(q4)', 's4'); str_out = strrep(str_out, 'cos(q4)', 'c4');
    str_out = strrep(str_out, 'sin(q5)', 's5'); str_out = strrep(str_out, 'cos(q5)', 'c5');
    str_out = strrep(str_out, 'sin(q6)', 's6'); str_out = strrep(str_out, 'cos(q6)', 'c6');
    % Dự phòng nếu còn sót hàm sin/cos phức tạp
    str_out = strrep(str_out, 'sin', 'np.sin');
    str_out = strrep(str_out, 'cos', 'np.cos');
end

% --- XUẤT MA TRẬN M (6x6) ---
fprintf(fid, '    # Mass Matrix M\n');
fprintf(fid, '    M = np.zeros((6,6))\n');
[rows, cols] = size(M);
for i = 1:rows
    for j = 1:cols
        % Chỉ ghi những phần tử khác 0
        if M(i,j) ~= 0
            expr = py_format(M(i,j));
            % Lưu ý: Python index từ 0, Matlab index từ 1
            fprintf(fid, '    M[%d, %d] = %s\n', i-1, j-1, expr);
        end
    end
end

% --- XUẤT MA TRẬN C (6x6) ---
fprintf(fid, '\n    # Coriolis Matrix C\n');
fprintf(fid, '    C = np.zeros((6,6))\n');
[rows, cols] = size(C);
for i = 1:rows
    for j = 1:cols
        if C(i,j) ~= 0
            expr = py_format(C(i,j));
            fprintf(fid, '    C[%d, %d] = %s\n', i-1, j-1, expr);
        end
    end
end

% --- XUẤT VECTOR G (6x1) ---
fprintf(fid, '\n    # Gravity Vector G\n');
fprintf(fid, '    G = np.zeros(6)\n');
for i = 1:6
    if G(i) ~= 0
        expr = py_format(G(i));
        fprintf(fid, '    G[%d] = %s\n', i-1, expr);
    end
end

fprintf(fid, '\n    return M, C, G\n');
fclose(fid);

disp('Đã xuất xong file ur10e_dynamics.py! Copy file này sang folder Python của bạn.');