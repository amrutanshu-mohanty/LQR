close all
% Parameters
Mm = 0.3; % Mass of motor
Mr = 0.7; % Mass of reaction wheel
Mb = 0.5; % Mass of pendulum
R = 0.25; % Radius of reaction wheel
lr = 0.5; % Distance of centre of reaction wheel from fixed base point, O
lb = 0.25; % Distance of centre of mass of pendulum from fixed base point, O

Imo = Mm*lr^2; % Moment of inertia of motor wrt O
Ir = Mr*R^2; % Moment of inertia of reaction wheel wrt its centre
Iro = Ir+ Mr*lr^2; % Moment of inertia of reaction wheel wrt O
Ibo = (Mb*lr^2)/3; % Moment of inertia of pendulum wrt O
Inet = Ibo + Iro + Imo; % Net moment of inertia of system wrt O

b = 0.2; % coefficient of viscous friction axle of pendulum
g = 9.8; % acceleration due to gravity

% initial points
theta_0 = 0;
theta_R_dot_0 = 0;

% set point

theta_req = 0.25;

% define the desired state 
xd=[0.4;0;0.3;0];



% A and B matrices

a = ((Mm+Mr)*lr + Mb*lb)*g;


A = [0 1 0 0;
    a/Inet -b/Inet 0 0;
    0 0 0 1;
    0 0 0 0];

B = [0 -1/Inet 0 1/Ir]';

C = eye(4);

D = 0;


% compute ud

ud=-inv(B'*B)*B'*A*xd;

% controllability
ctrb(A, B);
rank(ctrb(A,B));

% Q and R matrices

Q = [0.5 0 0 0;
    0 5 0 0;
    0 0 0.1 0;
    0 0 0 0.5];

R = 0.001;

% LQR controller

[K, S, P] = lqr(A, B, Q, R); % N is taken as 0 since it is omitted


% to design: 

% simulate closed loop system
% x = [theta, theta_dot, theta_R, theta_R_dot];
%u = -K*x;
sys = ss((A-B*K), B, C, D);
x0 = [0.7, 0, 0, 0];

% Run response
t = 0:0.05:5;
[y, t, x] = initial(sys, x0, t);


% Initial time and fuel values
current_time = 0;
fuel = 0;


% Plot

tiledlayout(3,2)
nexttile
plot(t,y(:,1));
title('Plot 1')
legend('\theta')
hold on
yline(xd(1))
hold off
nexttile
plot(t,y(:,2), 'g');
title('Plot 2')
legend('\omega')
hold on
yline(xd(2))
hold off
nexttile
plot(t,y(:,3), 'm');
title('Plot 3')
%p3.text(-2.5, 2.7, 'Fuel:            units', 'Color', [1 0 1], 'FontSize', 12);
legend('\theta_r')
hold on
yline(xd(3))
hold off
nexttile
plot(t,y(:,4), 'r');
title('Plot 4')
% Write time and fuel to the screen
%p4.text(-2.5, 3, 'Time:           sec', 'Color', [1 0 1], 'FontSize', 12);
legend('\omega_r')
hold on
yline(xd(4))
hold off
nexttile
plot(0)
text(0.1, 0.7, 'Time:           sec', 'Color', [0 0 0], 'FontSize', 9,...
'HorizontalAlignment', 'left');
TME = text(0.7, 0.7, num2str(current_time, '%.1f'), 'Color', [0 1 0], ...
'FontSize', 9, 'HorizontalAlignment', 'right');
nexttile
plot(0)
text(0.1, 0.7, 'Fuel:            units', 'Color', [0 0 0], 'FontSize', 9,...
'HorizontalAlignment', 'left');
FUL = text(0.7, 0.7, num2str(fuel, '%.0f'), 'Color', [1 0 0], ...
'FontSize', 9, 'HorizontalAlignment', 'right');

% Animation
figure;
for i = 1:1:length(x)-1
%for i = 1:length(y)
    
    if and(abs(x(i,1)) < 0.011, abs(x(i, 2)) < 0.011)
        x(i, 1) = 0;
        fprintf("time taken: ")
        str2double(current_time)
        fprintf("total fuel consumed: ")
        fuel*20
        pause(1);
        return
    end
    xa = lb * sin(y(i));
    ya = lb * cos(y(i));
    plot([0, xa], [0, ya], 'b', 'LineWidth', 2); % plots a line
    hold on;
    p = plot(xa, ya, 'ro', 'MarkerSize', 10); % plots the bob
    p.MarkerFaceColor = 'r';
    axis([-lr lr -lr lr]);
    title(['Inverted Pendulum Animation (t = ', num2str(t(i)), ' s)']);

    drawnow;
    pause(0.01);
    hold off;

    
    accel_scaled = (x(i+1, 2) - x(i, 2));
    
    fuel = fuel + abs(accel_scaled);
    
    accel_scaled = min(2, max(-2, accel_scaled * 1000));

    accel_true = diff(x(:, 4))/(t(2)-t(1));

    current_time = num2str(t(i), '%.1f');

    % Update time and fuel

    TME.String = current_time;
    FUL.String = num2str(fuel*20, '%.0f');

    pause(0.0001)

end

