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

figure;
hold on;
axis equal;
grid on;
axis([-5 20 -5 20]);
title('Robot LSPB MoveJ');
xlabel('X Position');
ylabel('Y Position');

plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8); %waypoint

robot_line = plot([0, 0], [0, 0], '-o', 'LineWidth', 2, 'MarkerSize', 5); %robot line


% moving with LSPB

current_pose = initial_pose;

for i = 1:size(waypoints,1)

    target_values = waypoints(i, :);
    
    final_pose = inverse_kinematics(target_values, l1, l2);
    
    [theta1_trajectory, theta2_trajectory] = LSPB_trajectory(current_pose, final_pose, time, dt);
    
    theta1 = [theta1, theta1_trajectory];
    theta2 = [theta2, theta2_trajectory];
    
    [x_trajectory, y_trajectory] = forward_kinematics(theta1_trajectory, theta2_trajectory, l1, l2);
    x = [x, x_trajectory];
    y = [y, y_trajectory];


    for j = 1:length(x_trajectory)
        set(robot_line, 'XData', [0, l1*cos(theta1_trajectory(j)), x_trajectory(j)], 'YData', [0, l1*sin(theta1_trajectory(j)), y_trajectory(j)]);
        pause(0.1);
    end
 
    current_pose = final_pose;
end


% config space trajectory

figure;
subplot(2,1,1);
plot(0:dt:(length(theta1)-1)*dt, theta1, 'b', 'LineWidth', 1.5);
hold on;
plot(0:dt:(length(theta2)-1)*dt, theta2, 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('Joint Angles (rad)');
title('Joint Trajectory (\theta_1, \theta_2)');
legend('\theta_1', '\theta_2');
grid on;


%  workspace trajectory

subplot(2,1,2);
plot(x, y, 'b-o', 'LineWidth', 1.5);
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('X');
ylabel('Y');
title('Workspace Trajectory (X, Y)');
grid on;








% IK function 
function theta = inverse_kinematics(target, l1, l2)
   
    x = target(1);
    y = target(2);
    
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    
    theta2_ik = atan2(-sqrt(1 - D^2), D);

    theta1_ik = atan2(y, x) - atan2(l2 * sin(theta2_ik), l1 + l2 * cos(theta2_ik));

    theta = [theta1_ik, theta2_ik];

    
    
end




% FK function
function [x_trajectory, y_trajectory] = forward_kinematics(theta1, theta2, l1, l2)

    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);
    
    x2 = x1 + l2 * cos(theta1 + theta2);
    y2 = y1 + l2 * sin(theta1 + theta2);

    x_trajectory = x2;
    y_trajectory = y2;
end




% LSPB

function [theta1_traj, theta2_traj] = LSPB_trajectory(q0, qf, tf, dt)
    
    q0_1 = q0(1);
    q0_2 = q0(2);
    qf_1 = qf(1);
    qf_2 = qf(2);

    
    tb = 0.333; % blending time = 1/3 (from textbook)
    V_1 = (qf_1 - q0_1) / (tf - tb);
    V_2 = (qf_2 - q0_2) / (tf - tb);
    V_peak_1 = V_1 * 1.5;
    V_peak_2 = V_2 * 1.5;


    t = 0:dt:tf;
    theta1_traj = zeros(size(t));
    theta2_traj = zeros(size(t));


    for i = 1:length(t)
        if t(i) < tb
            theta1_traj(i) = q0_1 + (V_peak_1 / (2*tb)) * t(i)^2;
            theta2_traj(i) = q0_2 + (V_peak_2 / (2*tb)) * t(i)^2;
        elseif t(i) < (tf - tb)
            theta1_traj(i) = q0_1 + V_peak_1 * (t(i) - tb / 2);
            theta2_traj(i) = q0_2 + V_peak_2 * (t(i) - tb / 2);
        else
            theta1_traj(i) = qf_1;
            theta2_traj(i) = qf_2;
        end
    end
end
