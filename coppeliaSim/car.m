function car(q, gamma_rad, scale, color)
    %% Sizing parameters
    l = 1*scale; % Distance from front wheels to rear wheels
    w = 0.75*scale; % Distance between wheels from left to right
    dfb = 0.5*scale; % Front-back distance from wheels to edge of car
    dlr = 0.2*scale; % Left-right distance from wheels to edge of car
    rwhl = 0.25*scale; % radius of a wheel
    wheelwidth = 4*scale; % Width of a wheel (in terms of LineWidth)
    
    x = q(1);
    y = q(2);
    theta_rad = q(3);
    
    %% Precalculation for faster computing time
    half_w = w/2; % This is more commonly used than w, so precalculate it here
%     theta_rad = deg2rad(theta_deg);
%     gamma_rad = deg2rad(gamma_deg);
    
    %% Body
    % Calculate the vertices of the car body relative to (x, y)
    x_body = zeros(1, 4); % x coordinates
    y_body = zeros(1, 4); % y coordinates
    dr = sqrt((half_w + dlr)^2 + dfb^2); % distance from (x, y) to rear corner of car
    df = sqrt((half_w + dlr)^2 + (l + dfb)^2); % distance from (x, y) to front corner of car
    [x_body(1), y_body(1)] = pol2cart(theta_rad + pi - atan((half_w + dlr)/dfb), dr);
    [x_body(2), y_body(2)] = pol2cart(theta_rad + pi + atan((half_w + dlr)/dfb), dr);
    [x_body(3), y_body(3)] = pol2cart(theta_rad - atan((half_w + dlr)/(l + dfb)), df);
    [x_body(4), y_body(4)] = pol2cart(theta_rad + atan((half_w + dlr)/(l + dfb)), df);
    
    % Draw the body of the car
    fill(x_body + x, y_body + y, color);
    hold on; % Make sure hold is turned on for the rest of the car parts
    
    %% Wheels
    % Calculate the center of the rear wheels relative to (x, y)
    [x_c_lrwhl, y_c_lrwhl] = pol2cart(theta_rad + pi/2, half_w);
    [x_c_rrwhl, y_c_rrwhl] = pol2cart(theta_rad - pi/2, half_w);
    
    % Get absolute coordinates
    x_c_lrwhl = x_c_lrwhl + x;
    x_c_rrwhl = x_c_rrwhl + x;
    y_c_lrwhl = y_c_lrwhl + y;
    y_c_rrwhl = y_c_rrwhl + y;
    
    % Calculate the endpoints of the rear wheels relative to their centers
    [x_rwhl(1), y_rwhl(1)] = pol2cart(theta_rad, rwhl);
    [x_rwhl(2), y_rwhl(2)] = pol2cart(theta_rad, -rwhl);
    
    % Draw the rear wheels
    line(x_rwhl + x_c_lrwhl, y_rwhl + y_c_lrwhl, 'Color', 'k', 'LineWidth', wheelwidth); % left wheel
    line(x_rwhl + x_c_rrwhl, y_rwhl + y_c_rrwhl, 'Color', 'k', 'LineWidth', wheelwidth); % right wheel
    
    % Find centers of front wheels by moving l from rear wheels
    x_c_lfwhl = x_c_lrwhl + l*cos(theta_rad);
    x_c_rfwhl = x_c_rrwhl + l*cos(theta_rad);
    y_c_lfwhl = y_c_lrwhl + l*sin(theta_rad);
    y_c_rfwhl = y_c_rrwhl + l*sin(theta_rad);
    
    % Calculate the endpoints of the front wheels relative to their centers
    [x_fwhl(1), y_fwhl(1)] = pol2cart(theta_rad + gamma_rad, rwhl);
    [x_fwhl(2), y_fwhl(2)] = pol2cart(theta_rad + gamma_rad, -rwhl);
    
    % Draw the front wheels
    line(x_fwhl + x_c_lfwhl, y_fwhl + y_c_lfwhl, 'Color', 'k', 'LineWidth', wheelwidth); % left wheel
    line(x_fwhl + x_c_rfwhl, y_fwhl + y_c_rfwhl, 'Color', 'k', 'LineWidth', wheelwidth); % right wheel
    
    %% Graphic details
    % Draw the axles
%     line([x_c_lfwhl x_c_rfwhl], [y_c_lfwhl y_c_rfwhl], 'Color', 'k'); % front axle
%     line([x_c_lrwhl x_c_rrwhl], [y_c_lrwhl y_c_rrwhl], 'Color', 'k'); % rear axle
    
    % Draw an arrow to indicate which direction the car is pointing
    if color == 'b'
        arrowColor = 'r';
    else
        arrowColor = 'b';
    end
    arrow([x + 0.5*scale*cos(theta_rad), y + 0.5*scale*sin(theta_rad), theta_rad], scale, arrowColor);
    
    % Draw a dot where the actual position is
    plot(x, y, '.k', 'MarkerSize', 10);
end