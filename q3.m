clc;
clear;

l1 = 11; % from HW 1
l2 = 10; % from HW 1

waypoints = [7, 7;  14, 7;  14, 15; 7, 15]; % form a rectangle

initial_pose = [pi/2, -pi/2]; % initial L pose


% given data
time = 1; % each position is 1 second apart
dt = 0.1; % sampling resolution

% initialize arrays
theta1 = [];
theta2 = [];
x = [];
y = [];


% robot figure

figure(1);
hold on; grid on; axis equal;
axis([-5 20 -5 20]);
title('MoveL with LSPB');
xlabel('X (m)'); ylabel('Y (m)');

plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8);    %waypoint

robot_line = plot([0, 0], [0, 0], '-o', 'LineWidth', 2, 'MarkerSize', 5);   %robot line

[current_x, current_y] = forward_kinematics(initial_pose(1), initial_pose(2), l1, l2);

current_position = [current_x, current_y];



% moving with LSPB

joint_position_1 = initial_pose;

for i = 1:size(waypoints,1)
    target_position = waypoints(i, :);

    [x_trajectory, y_trajectory] = LSPB(current_position, target_position, time, dt);
    

    theta1_trajectory = zeros(1, length(x_trajectory));
    theta2_trajectory = zeros(1, length(x_trajectory));

    for j = 1:length(x_trajectory)

        [theta1_trajectory(j), theta2_trajectory(j)] = inverse_kinematics([x_trajectory(j), y_trajectory(j)], l1, l2, joint_position_1);
        joint_position_1 = [theta1_trajectory(j), theta2_trajectory(j)];
    end

    theta1 = [theta1, theta1_trajectory];
    theta2 = [theta2, theta2_trajectory];
    x = [x, x_trajectory];
    y = [y, y_trajectory];


    for j = 1:length(x_trajectory)
        x1_joint = l1 * cos(theta1_trajectory(j));
        y1_joint = l1 * sin(theta1_trajectory(j));
        set(robot_line, 'XData', [0, x1_joint, x_trajectory(j)], 'YData', [0, y1_joint, y_trajectory(j)]);
        pause(0.1);
    end

    current_position = target_position;

end


% Workspace plot
figure(2);
plot(x, y, 'b-o', 'LineWidth', 1.5);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X'); ylabel('Y');
title('Workspace Trajectory');
grid on;


% Joint space plot
figure(3);
subplot(2,1,1);
t_joint = 0:dt:(length(theta1)-1)*dt;
plot(t_joint, theta1, 'b', 'LineWidth', 1.5);
hold on;
plot(t_joint, theta2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Joint Angles (rad)');
title('Joint Space Trajectory (\theta_1 and \theta_2)');
legend('\theta_1', '\theta_2');
grid on;





% IK Function
function [theta1, theta2] = inverse_kinematics(target, l1, l2, joint_position_0)
    x = target(1);
    y = target(2);

    D = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);

    theta2_up = atan2(sqrt(1-D^2), D); % arm elbow up position
    theta2_down = atan2(-sqrt(1-D^2), D); % arm elbow down position

    theta1_up = atan2(y, x) - atan2(l2*sin(theta2_up), l1 + l2*cos(theta2_up));
    theta1_down = atan2(y, x) - atan2(l2*sin(theta2_down), l1 + l2*cos(theta2_down));
    
    solution_up = [theta1_up, theta2_up];
    solution_down = [theta1_down, theta2_down];

    if norm(solution_up - joint_position_0) <= norm(solution_down - joint_position_0)
        theta1 = solution_up(1);
        theta2 = solution_up(2);
    else
        theta1 = solution_down(1);
        theta2 = solution_down(2);
    end
end



% FK Function
function [x, y] = forward_kinematics(theta1, theta2, l1, l2)
    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);
    x = x1 + l2 * cos(theta1 + theta2);
    y = y1 + l2 * sin(theta1 + theta2);
end



% LSPB 
function [x_trajectory, y_trajectory] = LSPB(p0, pf, tf, dt)

    tb = 0.333;  % Blend time = 1/3 (from textbook)

    V_avg_x = (pf(1) - p0(1)) / (tf - tb);
    V_avg_y = (pf(2) - p0(2)) / (tf - tb);

    V_peak_x = V_avg_x * 1.5; % peak V is 1.5x avg V
    V_peak_y = V_avg_y * 1.5; % peak V is 1.5x avg V

    t = 0:dt:tf;
    x_trajectory = zeros(size(t));
    y_trajectory = zeros(size(t));

    for i = 1:length(t)
        if t(i) < tb

            % acceleration 
            x_trajectory(i) = p0(1) + (V_peak_x/(2*tb)) * t(i)^2;
            y_trajectory(i) = p0(2) + (V_peak_y/(2*tb)) * t(i)^2;
        elseif t(i) <= (tf - tb)

            % constant velocity
            x_trajectory(i) = p0(1) + V_peak_x*(t(i) - tb/2);
            y_trajectory(i) = p0(2) + V_peak_y*(t(i) - tb/2);
        else
            % deceleration
            x_trajectory(i) = pf(1);
            y_trajectory(i) = pf(2);
        end
    end
end
