function diffDrive(q, scale, color)
    %% Sizing parameters
    r = 1*scale; % radius of robot
    wheelwidth = 3*scale; % Width of a wheel (in terms of LineWidth)
    
    x = q(1);
    y = q(2);
    theta = q(3);
    
    % Draw the body of the robot
    pos = [(x - r) (y - r) 2*r 2*r];
    rectangle('Position', pos, 'Curvature', [1, 1], 'FaceColor', color);
    
    % Find the endpoints of the wheels
    [x_lr, y_lr] = pol2cart(theta + 9*pi/12, 0.85*r);
    [x_rr, y_rr] = pol2cart(theta - 9*pi/12, 0.85*r);
    [x_lf, y_lf] = pol2cart(theta + 3*pi/12, 0.85*r);
    [x_rf, y_rf] = pol2cart(theta - 3*pi/12, 0.85*r);
    
    % Draw the wheels
    line([x_lr x_lf] + x, [y_lr y_lf] + y, 'Color', 'k', 'LineWidth', wheelwidth); % left wheel
    line([x_rr x_rf] + x, [y_rr y_rf] + y, 'Color', 'k', 'LineWidth', wheelwidth); % right wheel
    
    % Draw an arrow to indicate which direction the robot is pointing
    arrow(q, scale, 'b');    
end