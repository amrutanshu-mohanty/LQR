% Parameters
g = 9.81;       % Acceleration due to gravity (m/s^2)
L = 1.0;        % Pendulum length (m)
theta0 = pi/6;  % Initial angle (radians)
omega0 = 0;     % Initial angular velocity (rad/s)

% Time settings
tspan = [0 10]; % Simulation time span (seconds)
dt = 0.01;      % Time step (seconds)

% Numerical integration using ode45
[t, theta] = ode45(@(t, y) inverted_pendulum_dynamics(t, y, g, L), tspan, [theta0; omega0]);

% Animation
figure;
for i = 1:length(t)
    xa = L * sin(theta(i));
    ya = -L * cos(theta(i));
    plot([0, xa], [0, ya], 'b', 'LineWidth', 2); % plots a line
    hold on;
    plot(xa, ya, 'ro', 'MarkerSize', 10); % plots the bob
    axis([-L L -L L]);
    title(['Inverted Pendulum Animation (t = ', num2str(t(i)), ' s)']);
    drawnow;
    pause(dt);
    hold off;
end

function dydt = inverted_pendulum_dynamics(t, y, g, L)
    theta = y(1);
    omega = y(2);
    dydt = [omega; -g/L * sin(theta)];
end