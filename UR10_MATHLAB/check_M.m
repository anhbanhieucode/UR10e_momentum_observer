clear; clc;

% 1. Load dữ liệu
if isfile('ur10e_M_symbolic.mat')
    load('ur10e_M_symbolic.mat');
else
    error('Không tìm thấy file ur10e_M_symbolic.mat');
end

% 2. Chọn phần tử cần xem (Ví dụ M(1,3))
% Bạn có thể đổi số này tùy ý: M(1,1), M(2,2)...
Element = simplify(M(1,2));

% 3. Làm đẹp số liệu (Rút gọn về số thập phân 4 chữ số)
M_dep = vpa(Element, 4);

% 4. KIỂM TRA ĐỘ DÀI KÝ TỰ (BƯỚC QUAN TRỌNG)
% Chuyển sang dạng chuỗi văn bản để đếm
Str_M = char(M_dep); 
Len = length(Str_M);

fprintf('--- THÔNG TIN KIỂM TRA ---\n');
fprintf('Phần tử đang chọn: M(1,3)\n');
fprintf('Độ dài công thức: %d ký tự.\n', Len);

% 5. Quyết định có in hay không
Limit = 15000; % Giới hạn an toàn (khoảng 15-20k ký tự)

if Len < Limit
    fprintf('\n--- KẾT QUẢ HIỂN THỊ (PRETTY) ---\n\n');
    pretty(M_dep);
else
    fprintf('\n[CẢNH BÁO] Công thức quá dài (> %d ký tự)!\n', Limit);
    fprintf('Việc in ra màn hình có thể làm treo MATLAB.\n');
    fprintf('Gợi ý: Hãy dùng lệnh "latex(M_dep)" để lấy mã code và xem trong phần mềm khác.\n');
    
    % Tùy chọn: Vẫn in nếu người dùng muốn chấp nhận rủi ro
    % pretty(M_dep); 
end