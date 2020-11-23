function arrow(q, radius, color)
    % INPUTS:
    %   q - Pose of arrow [x; y; theta]
    %   radius - Radius of circle bounding the arrow
    %   color - Fill color of the arrow

    theta = q(3);
    
    x = zeros(4, 1);
    y = zeros(4, 1);
    
    [x(1), y(1)] = pol2cart(theta, radius);
    [x(2), y(2)] = pol2cart(theta + 5*pi/6, radius);
    [x(3), y(3)] = pol2cart(theta, -radius/2);
    [x(4), y(4)] = pol2cart(theta - 5*pi/6, radius);
    
    x = x + q(1);
    y = y + q(2);
    
    fill(x, y, color);
end