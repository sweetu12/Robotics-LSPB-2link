% function

function [final_pose] = function_Q1(target_position, home_pose, l1, l2, max_iterations, epsilon)
    
    theta1 = home_pose(1);
    theta2 = home_pose(2);

    lambda = 0.02;
    damping = 0.005;
    
    for i = 1:max_iterations

        x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
        y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
        current_position = [x, y];

        error = target_position - current_position;
        error_magnitude = norm(error);

        if error_magnitude < epsilon
            disp(['Took ', num2str(i), ' iterations.']);
            final_pose = [theta1, theta2];
            return;
        end

        J = [-l1 * sin(theta1) - l2 * sin(theta1 + theta2), -l2 * sin(theta1 + theta2);
              l1 * cos(theta1) + l2 * cos(theta1 + theta2),  l2 * cos(theta1 + theta2)];

        d_theta = (J' * (J * J' + damping * eye(2)) \ error') * lambda ;

        lambda = max(0.5, lambda * 0.99);

        theta1 = theta1 + lambda * d_theta(1);
        theta2 = theta2 + lambda * d_theta(2);
    end
   
    final_pose = [theta1, theta2];
end