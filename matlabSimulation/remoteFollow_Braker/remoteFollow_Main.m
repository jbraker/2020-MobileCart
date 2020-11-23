function remoteFollow_Main
clear; close all; clc;

%% Setup

% Landmarks: fixed RF sensor locations
lm = [-30, 10, 28, 30, 10, -20, -10;
      0.5, -20,-0.5, 19, 41, 39, 9];
  
% Initialize actual state of the robot
qTrue = zeros(3, 1); % [m, m, rad]
% Initialize estimated state of the robot
q = [0.1; 0.1; 0.05]; % [m, m, rad]

% Initialize state co-variance matrix 
P = diag((qTrue - q).^2);

% Initialize actual position of remote
qrTrue = [14; 22];
% Initialize estimated position of remote
qr = zeros(2, 1);

% Initialize other variables and constants
T = 0.25; % [s], Sampling time interval between control signals
ftag = 1:size(lm, 2); % Identifier for each landmark
da_table = zeros(1, size(lm, 2)); % Data association table
G = 0; % Initial steering angle  % control input

% Control parameters
V = 3; % [m/s], Constant linear velocity
MAXG = 30*pi/180; % [rad], Maximum steering angle (-MAXG < g < MAXG)
RATEG = 20*pi/180; % [rad/s], Maximum rate of change in steer angle
L = 4; % [m], Distance between front and rear wheels of robot

% Process (control) noise
sigmaV = 0.3; % [m/s], Linear speed standard deviation 
sigmaG = (3.0*pi/180); % [rad], Steering angle standard deviation
Q = [sigmaV^2 0; 0 sigmaG^2]; % Process noise covariance matrix

% Observation parameters
MAX_RANGE = 30.0; % [m], Maximum sensing distance
DT_OBSERVE = 8*T; % [s], Time interval between observations
dtsum = DT_OBSERVE; % [s], Change in time since last observation (Set to DT_OBSERVE to force observation on first iteration

% Observation noises
sigmaR = 0.1; % [m], Range standard deviation
sigmaB = (1.0*pi/180); % [rad], Bearing standard deviation
R = [sigmaR^2 0; 0 sigmaB^2]; % Observation noise covariance matrix

QE = Q;
RE = R;

% Data association innovation gates (Mahalanobis distances)
GATE_REJECT= 4.0; % maximum distance for association
GATE_AUGMENT= 25.0; % minimum distance for creation of new feature
% For 2-D observation:
%   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
%   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.

% Switches for noise
% If 0, no noise is added. If 1, noise is added
SWITCH_CONTROL_NOISE = 1; % if 0, velocity and gamma are perfect
SWITCH_SENSOR_NOISE = 1; % if 0, measurements are perfect
% Other switches
%SWITCH_ASSOCIATION_KNOWN = 0; % If 1, associations are given, if 0, they are estimated using gates
SWITCH_BATCH_UPDATE = 1; % If 1, process scan in batch, if 0, process sequentially

% Initial pose covariance estimate
P_init = diag([(qTrue(1)-q(1))^2;(qTrue(2)-q(2))^2;(qTrue(3)-q(3))^2]);

% Initialize vectors for recording values at each timestamp
QDT(:,1) = q(1:3); % Estimated robot pose
QTrueDT(:,1) = qTrue; % Actual robot pose
Pcov(:,:,1) = P_init; % Covariance
QRDT(:,1) = qr; % Estimated remote position

% Create figure
fig = figure;

% Create video writer
vid = VideoWriter('OUT/trajectory.avi');
vid.Quality = 100;
vid.FrameRate = 5; 
open(vid);

%% Main Loop
k = 1; % Current timestep
tf = 2; % Maximum simulation time
while not(simComplete(k, tf, T))
    % Clear the figure and plot the landmarks
    clf;
    hold on;
    axis(50*[-1 1 -1 1]); pbaspect([1 1 1]);
    plot(lm(1, :), lm(2, :), 'b*');
    xlabel('X [m]'); ylabel('Y [m]');
    
    %% Observations
    dtsum = dtsum + T;
    if dtsum >= DT_OBSERVE
        dtsum = 0;
        % Get the range-bearing observations for the landmarks and remote
        [z, ftag_visible, zr] = get_observations(qTrue, lm, ftag, qrTrue, MAX_RANGE);  % tag ID (ftag) is necessary if data association is known.
        % Add noise to the landmark observations
        z = add_observation_noise(z, R, SWITCH_SENSOR_NOISE);
        % Add noise to the remote observation
        zr = add_observation_noise(zr, R, SWITCH_SENSOR_NOISE);
        % Convert remote observation to Cartesian coordinates
        qr = [zr(1)*cos(zr(2)); zr(1)*sin(zr(2))];
        
        [zf, idf, zn]= data_associate(q, P, z, RE, GATE_REJECT, GATE_AUGMENT);               
        % zf = feature measurements with data association getReject < 4
        % zn = new feature measurements to be included.  distance > 25 [m]
        
        [q, P]= update(q, P, zf, RE, idf, SWITCH_BATCH_UPDATE);
        [q, P]= augment(q, P, zn, RE); 
    end
    
    % Advance timestep
    k = k + 1;
    
    %% Control inputs
    % Compute steering angle based on true pose
    G = compute_steering(qTrue, qr, G, RATEG, MAXG, T);
    qTrue = vehicle_model(qTrue, V, G, L, T);
    
    % Add process noise
    [Vn, Gn] = add_control_noise(V, G, Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step
    [q, P] = predict(q, P, Vn, Gn, QE, L, T);
    
    %% Animation
    % Plot the remote
    plot(qrTrue(1), qrTrue(2), 'b^', 'DisplayName', 'Actual remote position');
    plot(qrTrue(1), qrTrue(2), 'r^', 'DisplayName', 'Estimated remote position');
    % Plot the true and estimated robot positions
    arrow(qTrue, 3, 'b'); % True position
    arrow(q, 3, 'r'); % Estimated position
    % Save current information
    QDT(:,k) = q(1:3);
    QTrueDT(:,k) = qTrue;
    QRDT(:, k) = qr;
    plot(QDT(1,1:k), QDT(2,1:k),'r--', 'DisplayName', 'Estimated robot trajectory');
    plot(QTrueDT(1,1:k), QTrueDT(2,1:k),'b-');
    
    landmarkEstimate = plot(q(4:2:end), q(5:2:end),'r.','MarkerSize',16);
    
    
    fill(Xa,Ya,'r');
    fill(Xd,Yd,'b');
        
    if dtsum==0
        if ~isempty(z)
            plines= make_laser_lines (z,q(1:3));
            plot(plines(1,:),plines(2,:),'r-');            
            pcov= make_covariance_ellipses(q,P);
            [U1,S1,V1] = svd(P(1:3,1:3));
            ellipse(30*S1(1,1),30*S1(2,2),atan2(U1(1,2),U1(1,1)),QDT(1,i),QDT(2,i),'c');    
            %ellipse(S1(1,1),S1(2,2),atan2(U1(1,2),U1(1,1)),QDT(1,i),QDT(2,i),'r');               
            plot(pcov(1,:), pcov(2,:),'r')  % landmark pose covariance uncertainty ellipse
        end
    end
    drawnow
    F = getframe(fig);
    writeVideo(vid,F); 
end

% Close the video file
close(vid);
end

function done = simComplete(iter, tMax, T)
    % INPUTS:
    %   iter - Current iteration
    %   tMax - Maximum simulation time
    %   T - Discrete sampling time
    %
    % OUTPUTS:
    %   xv - new vehicle pose
    
    if T*iter >= tMax
        done = true;
    else
        done = false;
    end
end

function xv = vehicle_model(xv, V, G, L, T)
    % INPUTS:
    %   xv - Vehicle pose [x; y; theta]
    %   V - Velocity
    %   G - Steering angle (gamma)
    %   WB - Wheelbase
    %   T - Discrete sampling time
    %
    % OUTPUTS:
    %   xv - new vehicle pose

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% Why is sin used instead of tan in updating theta? %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xv = [xv(1) + V*T*cos(G+xv(3,:)); 
          xv(2) + V*T*sin(G+xv(3,:));
          wrapToPi(xv(3) + V*T*sin(G)/L)];
end

function [z, idf, zr] = get_observations(q, lm, idf, qr, rmax)
    % INPUTS:
    %   q - Vehicle pose [x; y; theta]
    %   lm - Positions of all landmarks
    %   idf - Index tags for each landmark
    %   qr - Position of remote
    %   rmax - Maximum range of range-bearing sensor 
    %
    % OUTPUTS:
    %   z - Set of range-bearing observations for landmarks
    %   idf - Landmark index tag for each observation
    %   zr - Range-bearing observation for remote
    %
    % Tim Bailey 2004.

    [lm, idf] = get_visible_landmarks(q, lm, idf, rmax);
    [z, zr] = compute_range_bearing(q, lm, qr);
end

function [lm, idf] = get_visible_landmarks(q, lm, idf, rmax)
    % INPUTS:
    %   q - Vehicle pose [x; y; theta]
    %   lm - Positions of all landmarks
    %   idf - Index tags for each landmark
    %   rmax - Maximum range of range-bearing sensor 
    %
    % OUTPUTS:
    %   lm - Positions of visible landmarks
    %   idf - Landmark index tag for each visible landmark
    
    % Select set of landmarks that are visible within vehicle's semi-circular field-of-view
    dx = lm(1,:) - q(1);
    dy = lm(2,:) - q(2);
    phi = q(3);

    % incremental tests for bounding semi-circle
    ii = find(abs(dx) < rmax & abs(dy) < rmax ... % bounding box
          & (dx*cos(phi) + dy*sin(phi)) > 0 ...  % bounding line  x' = dx*cos(phi) + dy*sin(phi) => x coordinate of the landmark w.r.to robot coordinate system
          & (dx.^2 + dy.^2) < rmax^2);           % bounding circle
    % Note: the bounding box test is unnecessary but illustrates a possible speedup technique
    % as it quickly eliminates distant points. Ordering the landmark set would make this operation
    % O(logN) rather that O(N).

    lm = lm(:,ii);
    idf = idf(ii);
end

function [z, zr] = compute_range_bearing(q, lm, qr)
    % INPUTS:
    %   q - Vehicle pose [x; y; theta]
    %   lm - Positions of visible landmarks
    %   idf - Index tags for each visible landmark
    %   qr - Position of remote [x; y]
    %
    % OUTPUTS:
    %   z - Range and bearing observations for each visible landmark
    %   zr - Range and bearing observation for remote
    
    %% Compute exact observation for landmarks
    dx = lm(1,:) - q(1);
    dy = lm(2,:) - q(2);
    theta = q(3);
    z = [sqrt(dx.^2 + dy.^2);   % [d1    d2    d3    ...
         atan2(dy,dx) - theta];   %  psi1  psi2  psi3  ...]
    
    %% Compute exact observation for remote
    dx = qr(1) - q(1);
    dy = qr(2) - q(2);
    theta = q(3);
    zr = [sqrt(dx.^2 + dy.^2);
          atan2(dy,dx) - theta];
end

function z = add_observation_noise(z, R, addnoise)
    % Add random measurement noise. We assume R is diagonal.    
    if addnoise == 1
        len = size(z,2);
        if len > 0
            z(1,:)= z(1,:) + randn(1,len)*sqrt(R(1,1));
            z(2,:)= z(2,:) + randn(1,len)*sqrt(R(2,2));
        end
    end
end

function [zf, idf, zn]= data_associate(x, P, z, R, gate1, gate2)
    % Simple gated nearest-neighbour data-association. No clever feature
    % caching tricks to speed up association, so computation is O(N), where
    % N is the number of features in the state.
    %
    % Tim Bailey 2004.

    zf = [];
    zn = [];
    idf = []; 

    Nxv = 3; % number of vehicle pose states
    Nf = (length(x) - Nxv)/2; % number of features already in map

    % linear search for nearest-neighbour, no clever tricks (like a quick
    % bounding-box threshold to remove distant features; or, better yet,
    % a balanced k-d tree lookup). TODO: implement clever tricks.
    for i = 1:size(z,2)
        jbest = 0;
        nbest = inf;
        outer = inf;

        % search for neighbours
        for j = 1:Nf
            [nis, nd]= compute_association(x, P, z(:,i), R, j);
            if nis < gate1 && nd < nbest % if within gate, store nearest-neighbour
                nbest = nd;
                jbest = j;
            elseif nis < outer % else store best nis value
                outer = nis;
            end
        end

        % add nearest-neighbour to association list
        if jbest ~= 0
            zf =  [zf  z(:,i)];
            idf = [idf jbest];
        elseif outer > gate2 % z too far to associate, but far enough to be a new feature
            zn = [zn z(:,i)];
        end
    end
end

function [nis, nd]= compute_association(x,P,z,R,idf)
    % Return normalised innovation squared (ie, Mahalanobis distance) and normalised distance
    [zp, H] = observe_model(x, idf);  % here idf is the index of a single feature
    v = z - zp; 
    v(2) = wrapToPi(v(2));
    S = H*P*H' + R;

    nis = v'*inv(S)*v;
    nd = nis + log(det(S));
end

function [q, P]= update(q, P, z, R, idf, batch)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM state covariance
    %   z - Range-bearing measurements
    %   R - Range-bearing covariances
    %   idf - Feature index for each z
    %   batch - Switch to specify whether to process measurements together or sequentially
    %
    % Outputs:
    %   x - Updated state
    %   P - Updated covariance

    if batch == 1
        [q, P] = batch_update(q, P, z, R, idf);
    else
        [q, P] = single_update(q, P, z, R, idf);
    end
end

function [q, P]= batch_update(q, P, z, R, idf)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM state covariance
    %   z - Range-bearing measurements
    %   R - Range-bearing covariances
    %   idf - Feature index for each z
    %
    % Outputs:
    %   x - Updated state
    %   P - Updated covariance
    
    lenz = size(z,2);
    lenx = length(q);
    H = zeros(2*lenz, lenx);
    v = zeros(2*lenz, 1);
    RR = zeros(2*lenz);

    for i = 1:lenz
        ii = 2*i + (-1:0);
        [zp, H(ii,:)] = observe_model(q, idf(i));

        v(ii)= [z(1,i) - zp(1);
                wrapToPi(z(2,i) - zp(2))];
        RR(ii,ii)= R;
    end

    [q, P] = KF_cholesky_update(q, P, v, RR, H);
end

function [q,P]= single_update(q, P, z, R, idf)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM covariance
    %   z - Range-bearing measurements
    %   R - Range-bearing covariances
    %   idf - Feature index for each z
    %
    % Outputs:
    %   x - Updated state
    %   P - Updated covariance
    
    lenz = size(z,2);
    for i = 1:lenz
        [zp, H] = observe_model(q, idf(i));

        v = [z(1,i) - zp(1);
            wrapToPi(z(2,i) - zp(2))];

        [q, P]= KF_cholesky_update(q, P, v, RR, H);
    end
end

function [z,H]= observe_model(x, idf)
    % INPUTS:
    %   x - state vector
    %   idf - index of feature order in state
    %
    % OUTPUTS:
    %   z - predicted observation
    %   H - observation Jacobian
    %
    % Given a feature index (ie, the order of the feature in the state vector),
    % predict the expected range-bearing observation of this feature and its Jacobian.
    %
    % Tim Bailey 2004.

    Nxv = 3; % Number of vehicle pose states
    fpos = Nxv + idf*2 - 1; % Position of xf in state
    H = zeros(2, length(x));

    % Auxiliary values
    dx = x(fpos) - x(1); 
    dy = x(fpos+1) - x(2);
    d2 = dx^2 + dy^2;
    d = sqrt(d2);
    xd = dx/d;
    yd = dy/d;
    xd2 = dx/d2;
    yd2 = dy/d2;

    % Predict z
    z = [d;
        atan2(dy,dx) - x(3)];

    % Calculate H
    H(:,1:3)        = [-xd -yd 0; yd2 -xd2 -1];
    H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];
end

function [q, P]= KF_cholesky_update(q, P, v, R, H)
    % Calculate the KF (or EKF) update given the prior state [x, P]
    % the innovation [v, R] and the (linearised) observation model H.
    % The result is calculated using Cholesky factorization, which
    % is more numerically stable than a naive implementation.
    %
    % Tim Bailey 2003
    % Developed by Jose Guivant 

    PHt = P*H';
    S = H*PHt + R;

    S = (S+S')*0.5; % make symmetric
    SChol = chol(S);

    SCholInv = inv(SChol); % triangular matrix
    W1 = PHt * SCholInv;
    W = W1 * SCholInv';

    q = q + W*v; % update 
    P = P - W1*W1';
end

function [q, P]= augment(q, P, z, R)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM covariance
    %   z - Range-bearing measurements of a new feature
    %   R - Range-bearing covariances of a new feature
    %
    % Outputs:
    %   x - Augmented state 
    %   P - Augmented covariance
    %
    % Notes: 
    %   - We assume the number of vehicle pose states is three.
    %   - Only one value for R is used, as all measurements are assumed to 
    %   have same noise properties.
    %
    % Tim Bailey 2004.

    % add new features to state
    for i = 1:size(z,2)
        [q, P] = add_one_z(q, P, z(:,i), R);
    end
end

function [q, P]= add_one_z(q, P, z, R)
    len = length(q);
    r = z(1);
    b = z(2);
    s = sin(q(3) + b); 
    c = cos(q(3) + b);

    % Augment q
    q = [q;
         q(1) + r*c;
         q(2) + r*s];

    % Jacobians
    Gv = [1 0 -r*s;
          0 1  r*c];
    Gz = [c -r*s;
          s  r*c];

    % Augment P
    rng = len+1:len+2;
    P(rng,rng) = Gv*P(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
    P(rng,1:3) = Gv*P(1:3,1:3); % vehicle to feature xcorr
    P(1:3,rng) = P(rng,1:3)';
    if len > 3
        rnm = 4:len;
        P(rng,rnm) = Gv*P(1:3,rnm); % map to feature xcorr
        P(rnm,rng) = P(rng,rnm)';
    end
end

function G = compute_steering(q, qr, G, rateG, maxG, dt)
    % INPUTS:
    %   q - True position of robot
    %   qr - True position of remote
    %   G - Current steering angle
    %   rateG - Max steering rate (rad/s)
    %   maxG - Max steering angle (rad)
    %   dt - Timestep
    %
    % OUTPUTS:
    %   G - New steering angle
    
    % Compute change in G (deltaG) to point towards remote
    % deltaG = desired angle - robot's actual heading angle - robot's steering angle
    deltaG = wrapToPi(atan2(qr(2) - q(2), qr(1) - q(1)) - q(3) - G);  

    % Limit steering rate
    maxDelta = rateG*dt;
    if abs(deltaG) > maxDelta
        deltaG = sign(deltaG)*maxDelta;
    end

    % Limit steering angle
    G = G + deltaG;
    if abs(G) > maxG
        G = sign(G)*maxG;
    end
end

function [V, G]= add_control_noise(V, G, Q, addnoise)
    % INPUTS:
    %   V - Linear velocity
    %   G - Steering angle
    %   Q - Covariance
    %   addnoise - Flag to indicate whether to add noise or not
    %
    % OUTPUTS:
    %   V - Noisy linear velocity
    %   G - Noisy steering angle

    % Add random noise to nominal control values. We assume Q is diagonal.
    if addnoise == 1
        V = V + randn(1)*sqrt(Q(1,1));
        G = G + randn(1)*sqrt(Q(2,2));
    end
end

function [q, P] = predict(q, P, v, g, Q, L, T)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM covariance
    %   v - Linear velocity
    %   g - Steering angle
    %   Q - covariance matrix for velocity and gamma
    %   L - Distance between front and rear wheels of robot
    %   T - Sample time
    %
    % Outputs: 
    %   xn - Predicted state
    %   Pn - Predicted covariance
    %
    % Tim Bailey 2004.

    s = sin(g + q(3));
    c = cos(g + q(3));
    vts = v*T*s;
    vtc = v*T*c;

    % Jacobians   
    Fk = [1 0 -vts;
          0 1  vtc;
          0 0 1];
    Lk = [T*c -vts;
          T*s  vtc;
          T*sin(g)/L v*T*cos(g)/L];

    % Predict covariance
    P(1:3,1:3) = Fk*P(1:3,1:3)*Fk' + Lk*Q*Lk';
    if size(P,1) > 3
        P(1:3, 4:end) = Fk*P(1:3, 4:end);
        P(4:end, 1:3) = P(1:3, 4:end)';
    end    

    % Predict state
    q(1:3) = [q(1) + vtc; 
              q(2) + vts;
              wrapToPi(q(3) + v*T*sin(g)/L)];
end