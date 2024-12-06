r1 = 10;    % Link radius
r2 = 15;    % Joint radius
r3 = 25;    % Base radius
h_base = 20;
h_joint = 40;

theta1 = deg2rad(90);
theta2 = deg2rad(-45);
theta3 = deg2rad(60);

d1 = 80;
a2 = 60;
a3 = 50;

% Tao khoi O_0
[x1, y1, z1] = cylinder(r3);        % Base O_0
z1 = z1 * h_base;
[x2, y2, z2] = cylinder(r2);        % Joint O_0
z2 = z2 * h_joint;
[x3, y3, z3] = cylinder(r1);        % Link O_0
z3 = z3 * d1;

% Tao khoi O_1
[x4, y4, z4] = cylinder(r2);        % Joint O_1
z4 = z4 * h_joint;
[x5, y5, z5] = cylinder(r1);        % Link O_1

% Tao khoi O_2
[x6, y6, z6] = cylinder(r2);        % Joint O_2
z6 = z6 * h_joint;
[x7, y7, z7] = cylinder(r1);        % Link O_2

x_e = [20,30,40,50,60,60,50,40,40,40,50,60,60,50,40,30,20,20,20,20,20,20];
x_e = x_e - 20;
z_e = [-30,-30,-30,-30,-20,-10,-10,-10,0,10,10,10,20,30,30,30,30,20,10,0,-10,-20];
y_e = 10*ones(1,size(x_e,2));
y_e(2,:)= -10*ones(1,size(x_e,2));

x_e = repmat(x_e, 2, 1);
z_e = repmat(z_e, 2, 1); 

H = [x_e(1,:); y_e(1,:); z_e(1,:); ones(1,size(x_e,2))];
R = [x_e(2,:); y_e(2,:); z_e(2,:); ones(1,size(x_e,2))];

% Xoay 90 do => link vuong goc joint
rotY = [cos(pi/2)  0 sin(pi/2) 0; 
        0          1 0         0; 
        -sin(pi/2) 0 cos(pi/2) 0; 
        0          0 0         1];

% Transformation matrices
T0_0 = eye(4);
T0_1 = T(d1, theta1, 0, pi/2);
T1_2 = T(0, theta2, a2, 0);
T2_3 = T(0, theta3, a3, 0);

T0_2 = T0_1 * T1_2;
T0_3 = T0_2 * T2_3;
 
joint0 = T0_0(1:3, 4);  % Base origin
joint1 = T0_1(1:3, 4); % Joint 1
joint2 = T0_2(1:3, 4); % Joint 2
joint3 = T0_3(1:3, 4); % Joint 3 (end effector)

% Joint 1
for j = 1:length(x2)  
    for i = 1:2  
        Pos2 = [x2(i, j); y2(i, j); z2(i, j) - h_joint/2 ; 1];          % Joint O_0 
        Pos4 = T0_1 * Pos2;                                             % Joint O_1
        x4(i, j) = Pos4(1);
        y4(i, j) = Pos4(2);
        z4(i, j) = Pos4(3);
    end
end

% Link 1
for j = 1:length(x3)  
    for i = 1:2  
        Pos3 = [x3(i, j); y3(i, j); -z3(i, j)*a2/d1; 1];                % Link O_0
        Pos5 = T0_2 * rotY * Pos3;                                      % Link O_1
        x5(i, j) = Pos5(1);
        y5(i, j) = Pos5(2);
        z5(i, j) = Pos5(3);
    end
end

% Joint 2
for j = 1:length(x2)  
    for i = 1:2  
        Pos2 = [x2(i, j); y2(i, j); z2(i, j) - h_joint/2 ; 1];          % Joint O_0 
        Pos6 = T0_2 * Pos2;                                             % Joint O_2
        x6(i, j) = Pos6(1);
        y6(i, j) = Pos6(2);
        z6(i, j) = Pos6(3);
    end
end

% Link 2
for j = 1:length(x3)  
    for i = 1:2  
        Pos3 = [x3(i, j); y3(i, j); -z3(i, j)*a3/d1; 1];                % Link O_0
        Pos7 = T0_3 * rotY * Pos3;                                      % Link O_2
        x7(i, j) = Pos7(1);
        y7(i, j) = Pos7(2);
        z7(i, j) = Pos7(3);
    end
end

%End effector
H_new = T0_3*R;
R_new = T0_3*H;
x_e(1,:) = H_new(1,:);x_e(2,:) = R_new(1,:);
y_e(1,:) = H_new(2,:);y_e(2,:) = R_new(2,:);
z_e(1,:) = H_new(3,:);z_e(2,:) = R_new(3,:);

figure;
hold on;
grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robotics');

% �?nh sáng và shading
shading interp;        % Làm mịn b�? mặt
% lighting gouraud;      % Dùng phương pháp Gouraud để hiển thị ánh sáng
% camlight;              % �?ặt nguồn sáng theo góc nhìn của camera

% Base
surf(x1, y1, z1, 'EdgeColor', 'none', 'FaceColor', 'r'); 
fill3(x1(1,:), y1(1,:), z1(1,:), 'r');
fill3(x1(2,:), y1(2,:), z1(2,:), 'r');

% Joint 0 - Blue
surf(x2, y2, z2, 'EdgeColor', 'none', 'FaceColor', 'b');
fill3(x2(1,:), y2(1,:), z2(1,:), 'b');
fill3(x2(2,:), y2(2,:), z2(2,:), 'b');

% Link 0 - Yellow
surf(x3, y3, z3, 'EdgeColor', 'none', 'FaceColor', 'y');
fill3(x3(1,:), y3(1,:), z3(1,:), 'y');
fill3(x3(2,:), y3(2,:), z3(2,:), 'y');

% Joint 1 - Blue
surf(x4, y4, z4, 'EdgeColor', 'none', 'FaceColor', 'b');
fill3(x4(1,:), y4(1,:), z4(1,:), 'b');
fill3(x4(2,:), y4(2,:), z4(2,:), 'b');

% Link 1 - Yellow
surf(x5, y5, z5, 'EdgeColor', 'none', 'FaceColor', 'y');
fill3(x5(1,:), y5(1,:), z5(1,:), 'y');
fill3(x5(2,:), y5(2,:), z5(2,:), 'y');

% Joint 2 - Blue
surf(x6, y6, z6, 'EdgeColor', 'none', 'FaceColor', 'b');
fill3(x6(1,:), y6(1,:), z6(1,:), 'b');
fill3(x6(2,:), y6(2,:), z6(2,:), 'b');

% Link 2 - Yellow
surf(x7, y7, z7, 'EdgeColor', 'none', 'FaceColor', 'y');
fill3(x7(1,:), y7(1,:), z7(1,:), 'y');
fill3(x7(2,:), y7(2,:), z7(2,:), 'y');

%End effector
surf(x_e, y_e, z_e, 'FaceColor', 'r');
fill3(x_e(1, :), y_e(1,:), z_e(1,:),'g');
fill3(x_e(2, :), y_e(2,:), z_e(2,:),'g');


% �?i�?u chỉnh không gian t�?a độ
% xlim([-80 80]);
% ylim([-80 80]);
% zlim([0 120]);

% Bật chế độ xoay 3D
rotate3d on;
hold off;