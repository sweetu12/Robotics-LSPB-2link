clc; 
clear;

l1 = 11; % from hw1
l2 = 10;


waypoints = [7, 7;   14, 7;  14, 15; 7, 15]; % rectangle

home_pose = [pi/2, -pi/2];  % L shape



for t = 1:size(waypoints, 1)

    target_position = waypoints(t, :);
    
    final_pose = function_Q1(target_position, home_pose, l1, l2, 200000, 0.1);
    
    
    theta1 = final_pose(1);
    theta2 = final_pose(2);
    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);
    x2 = x1 + l2 * cos(theta1 + theta2);
    y2 = y1 + l2 * sin(theta1 + theta2);


    home_pose = final_pose;
end

