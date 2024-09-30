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

% A and B matrices

a = ((Mm+Mr)*lr + Mb*lb)*g;


A = [0 1 0 0;
    a/Inet -b/Inet 0 0;
    0 0 0 1;
    0 0 0 0];

B = [0 -1/Inet 0 1/Ir]';

C = eye(4);

D = 0;

% controllability
ctrb(A, B);
rank(ctrb(A,B));

% Q and R matrices
%Q = eye(4);
Q = [3 0 0 0;
    0 2 0 0;
    0 0 1 0;
    0 0 0 5];
R = 2;

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

% Plot

tiledlayout(2,2)
nexttile
plot(t,y(:,1))
title('Plot 1')
legend('\theta')
nexttile
plot(t,y(:,2), 'g')
title('Plot 2')
legend('\omega')
nexttile
plot(t,y(:,3), 'm')
title('Plot 3')
legend('\theta_r')
nexttile
plot(t,y(:,4), 'r')
title('Plot 4')
legend('\omega_r')

% Animation
figure;
for i = 1:length(y)
    [y, t, x] = initial(sys, x0, t);
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
end
