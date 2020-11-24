function remoteFollow_Main
clear; close all; clc;

%% Setup
T = 0.25; % [s], Sampling time interval between control signals

% Waypoints for the remote to follow
wp = [4, 14.5, 8, -10 -18 22 4;
      4, 8, 22, 29 -6 -8 4];
% Remote trajectory
remTraj = [];
vr = 1; % [m/s] Remote velocity
for ii = 1:(size(wp, 2) - 1)
    d = norm(wp(:, ii+1) - wp(:, ii));
    tseg = d/vr;
    segSteps = floor(tseg/T);
    m = (wp(2, ii+1) - wp(2, ii))/(wp(1, ii+1) - wp(1, ii));
    b = wp(2, ii) - m*wp(1, ii);
    x = linspace(wp(1, ii), wp(1, ii+1), segSteps);
    remTraj = [remTraj [x; (m*x + b)]];
end
% Initial actual position of remote
qrTrue = remTraj(:, 1);
numSteps = size(remTraj, 2);

% Landmarks: fixed RF sensor locations
lm = [-30, 10, 28, 30, 10, -20, -10;
      0.5, -20,-0.5, 19, 41, 39, 9];
  
% Initialize actual state of the robot
qTrue = zeros(3, 1); % [m, m, rad]
% Initialize estimated state of the robot
q = [0.1; 0.1; 0.05]; % [m, m, rad]

% Initialize state co-variance matrix
P = diag((qTrue - q).^2);

% Initialize other variables and constants
ftag = 1:size(lm, 2); % Identifier for each landmark
da_table = zeros(1, size(lm, 2)); % Data association table

% Control parameters
MAXV = 1; % [m/s], Maximum linear velocity (-MAXV < v < MAXV)
KV = 1;
MAXW = 20*pi/180; % [rad], Maximum angular velocity (-MAXW < w < MAXW)
KW = 2; % Proportional gain for angular velocity

% Process (control) noise
sigmaV = 0.3; % [m/s], Linear speed standard deviation
sigmaW = (3.0*pi/180); % [rad], Steering angle standard deviation
Q = [sigmaV^2 0; 0 sigmaW^2]; % Process noise covariance matrix

% Observation parameters
MAX_RANGE = 30.0; % [m], Maximum sensing distance
DT_OBSERVE = 2*T; % [s], Time interval between observations
dtsum = 0; % [s], Change in time since last observation (Set to DT_OBSERVE to force observation on first iteration

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
P_init = diag([(qTrue(1)-q(1))^2; (qTrue(2)-q(2))^2; (qTrue(3)-q(3))^2]);

df = 1.5; % [m], Following distance

% Initial observation
% Get the range-bearing observations for the landmarks and remote
[~, ~, zr] = get_observations(qTrue, lm, ftag, qrTrue, MAX_RANGE);  % tag ID (ftag) is necessary if data association is known.
% Add noise to the remote observation
zr = add_observation_noise(zr, R, SWITCH_SENSOR_NOISE);
% Convert remote observation to Cartesian coordinates
qr = [zr(1)*cos(zr(2) + q(3)) + q(1); zr(1)*sin(zr(2) + q(3)) + q(2)];
% Calculate the target point (1 m closer than the remote)
qt = [(zr(1) - df)*cos(zr(2) + q(3)) + q(1); (zr(1) - df)*sin(zr(2) + q(3)) + q(2)];

% Initialize vectors for recording values at each timestamp
QDT(:,1) = q(1:3); % Estimated robot pose
QTrueDT(:,1) = qTrue; % Actual robot pose
Pcov(:,:,1) = P_init; % Covariance
QRDT(:,1) = qr; % Estimated remote position
QRTrueDT(:,1) = qrTrue; % Actual remote position

% Create figure
fig = figure;

% Create video writer
vid = VideoWriter('OUT/trajectory.mp4', 'MPEG-4');
vid.Quality = 100;
vid.FrameRate = 10; 
open(vid);

%% Main Loop
k = 1; % Current timestep
while not(simComplete(k, 2*numSteps))
    % Clear the figure and plot the landmarks
    clf;
    hold on;
    axis([-50 50 -40 60]); pbaspect([1 1 1]);
    plot(lm(1, :), lm(2, :), 'b*');
    xlabel('X [m]'); ylabel('Y [m]');
    
    % Update remote position
    qrTrue = remTraj(:, mod(k-1, numSteps)+1);
    
    % Advance timestep
    k = k + 1;
    
    %% Control inputs
    % Compute control inputs based on true pose
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       Should this use q or qTrue?                      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [v, w] = compute_control(q, qt, KV, KW, MAXV, MAXW);
    qTrue = vehicle_model(qTrue, v, w, T);
    
    % Add process noise
    [vn, wn] = add_control_noise(v, w, Q, SWITCH_CONTROL_NOISE);
    
    %% EKF predict step
    [q, P] = predict(q, P, vn, wn, QE, T);
    
    %% EKF Update
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
        qr = [zr(1)*cos(zr(2) + q(3)) + q(1); zr(1)*sin(zr(2) + q(3)) + q(2)];
        % Calculate the target point (1 m closer than the remote)
        qt = [(zr(1) - df)*cos(zr(2) + q(3)) + q(1); (zr(1) - df)*sin(zr(2) + q(3)) + q(2)];
        
        [zf, idf, zn]= data_associate(q, P, z, RE, GATE_REJECT, GATE_AUGMENT);
        % zf = feature measurements with data association getReject < 4
        % zn = new feature measurements to be included.  distance > 25 [m]
        
        [q, P] = update(q, P, zf, RE, idf, SWITCH_BATCH_UPDATE);
        [q, P] = augment(q, P, zn, RE);
    end
    
    %% Animation
    % Save current information
    QDT(:,k) = q(1:3);
    QTrueDT(:,k) = qTrue;
    QRDT(:, k) = qr;
    QRTrueDT(:, k) = qrTrue;
    
    % Plot the remote
    plot(remTraj(1, :), remTraj(2, :), '--', 'Color', 0.85*[1 1 1], 'DisplayName', 'Remote trajectory');
    plot(qrTrue(1), qrTrue(2), 'b^', 'DisplayName', 'Actual remote position');
    plot(qr(1), qr(2), 'r^', 'DisplayName', 'Estimated remote position');
    % Plot the trajectory
    plot(QDT(1,1:k), QDT(2,1:k),'r--', 'DisplayName', 'Estimated robot trajectory');
    plot(QTrueDT(1,1:k), QTrueDT(2,1:k),'b-', 'DisplayName', 'Actual robot trajectory');
    % Plot the true and estimated robot positions
    arrow(qTrue, 2, 'b'); % True position
    arrow(q, 2, 'r'); % Estimated position
    
    % Plot the estimated landmark positions
    plot(q(4:2:end), q(5:2:end), 'r.', 'MarkerSize', 16, 'DisplayName', 'Estimated landmark positions');
        
    if dtsum == 0
        if ~isempty(z)
            plines= make_laser_lines (z,q(1:3));
            plot(plines(1,:),plines(2,:),'r-');            
            pcov = make_covariance_ellipses(q,P);
            [U1, S1, V1] = svd(P(1:3,1:3));
            ellipse(30*S1(1,1), 30*S1(2,2), atan2(U1(1,2), U1(1,1)), QDT(1,k), QDT(2,k), 'c');    
            %ellipse(S1(1,1),S1(2,2),atan2(U1(1,2),U1(1,1)),QDT(1,i),QDT(2,i),'r');               
            plot(pcov(1,:), pcov(2,:),'r')  % landmark pose covariance uncertainty ellipse
        end
    end
    drawnow;
    F = getframe(fig);
    writeVideo(vid,F);
end

% Close the video file
close(vid);
end

function done = simComplete(iter, maxIter)
    % INPUTS:
    %   iter - Current iteration
    %   maxIter - Maximum number of iterations
    %
    % OUTPUTS:
    %   done - Boolean to indicate whether to end simulation
    
    if iter >= maxIter
        done = true;
    else
        done = false;
    end
end

function q = vehicle_model(q, v, w, T)
    % INPUTS:
    %   q - Vehicle pose [x; y; theta]
    %   v - Linear velocity
    %   w - Angular velocity
    %   T - Discrete sampling time
    %
    % OUTPUTS:
    %   q - new vehicle pose

    q = [q(1) + T*v*cos(q(3)); 
         q(2) + T*v*sin(q(3));
         q(3) + T*w];
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

function [v, w] = compute_control(q, qt, KV, KW, maxV, maxW)
    % INPUTS:
    %   q - True position of robot
    %   qt - True position of target point
    %   KV - Proportional gain for v
    %   KW - Proportional gain for w
    %   maxV - Max linear velocity [m/s]
    %   maxW - Max angular velocity [rad/s]
    %   dt - Timestep
    %
    % OUTPUTS:
    %   v - Linear velocity
    %   w - Angular velocity
    
    % Determine linear velocity
    v = sign(cos(atan2(qt(2) - q(2), qt(1) - q(1)) - q(3)))*KV*norm(qt - q(1:2));
    % Limit linear velocity
    v = sign(v)*min(maxV, abs(v));
    
    % Compute angle to remote
    thetaRef = atan2(qt(2) - q(2), qt(1) - q(1));

    % Determine angular velocity
    w = KW*wrapToPi(thetaRef - q(3));
    % Limit angular velocity
    w = sign(w)*min(abs(w), maxW);
end

function [v, w]= add_control_noise(v, w, Q, addnoise)
    % INPUTS:
    %   v - Linear velocity
    %   w - Angular velocity
    %   Q - Covariance
    %   addnoise - Flag to indicate whether to add noise or not
    %
    % OUTPUTS:
    %   v - Noisy linear velocity
    %   w - Noisy angular velocity

    % Add random noise to nominal control values. We assume Q is diagonal.
    if addnoise == 1
        v = v + randn(1)*sqrt(Q(1,1));
        w = w + randn(1)*sqrt(Q(2,2));
    end
end

function [q, P] = predict(q, P, v, w, Q, T)
    % Inputs:
    %   q - SLAM state
    %   P - SLAM covariance
    %   v - Linear velocity
    %   w - Angular velocity
    %   Q - covariance matrix for velocity and gamma
    %   T - Sample time
    %
    % Outputs: 
    %   q - Predicted state
    %   P - Predicted covariance
    %
    % Tim Bailey 2004.
    
    x = q(1);
    y = q(2);
    theta = q(3);

    s = sin(theta);
    c = cos(theta);
    vts = v*T*s;
    vtc = v*T*c;

    % Jacobians
    Fk = [1 0 -vts;
          0 1  vtc;
          0 0 1];
    
    Lk = [T*c 0;
          T*s 0;
          0   T];

    % Predict covariance
    P(1:3,1:3) = Fk*P(1:3,1:3)*Fk' + Lk*Q*Lk';
    if size(P,1) > 3
        P(1:3, 4:end) = Fk*P(1:3, 4:end);
        P(4:end, 1:3) = P(1:3, 4:end)';
    end    

    % Predict state
    q(1:3) = [x + vtc; 
              y + vts;
              wrapToPi(theta + T*w)];
end

function p = make_laser_lines (rb,xv)
    % Compute set of line segments for laser range-bearing measurements
    if isempty(rb)
        p = [];
        return
    end
    len = size(rb,2);
    lnes(1,:) = zeros(1,len)+ xv(1);
    lnes(2,:) = zeros(1,len)+ xv(2);
    lnes(3:4,:) = TransformToGlobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
    p = line_plot_conversion (lnes);
end

function p = TransformToGlobal(p, b)
    % Transform a list of poses [x;y;phi] so that they are global wrt a base pose
    %
    % Tim Bailey 1999

    % rotate
    rot = [cos(b(3)) -sin(b(3)); sin(b(3)) cos(b(3))];
    p(1:2,:) = rot*p(1:2,:);

    % translate
    p(1,:) = p(1,:) + b(1);
    p(2,:) = p(2,:) + b(2);

    % if p is a pose and not a point
    if size(p,1)==3
       p(3,:) = pi_to_pi(p(3,:) + b(3));
    end
end

function p= line_plot_conversion (lne)
    % INPUT: list of lines [x1;y1;x2;y2]
    % OUTPUT: list of points [x;y]
    %
    % Convert a list of lines so that they will be plotted as a set of
    % unconnected lines but only require a single handle to do so. This
    % is performed by converting the lines to a set of points, where a
    % NaN point is inserted between every point-pair:
    %
    %   l= [x1a x1b x1c;
    %       y1a y1b y1c;
    %       x2a x2b x2c;
    %       y2a y2b y2c];
    %
    %   becomes
    %
    %   p= [x1a x2a NaN x1b x2b NaN x1c x2c;
    %       y1a y2a NaN y1b y2b NaN y1c y2c];
    %
    % Tim Bailey 2002. Thanks to Jose Guivant for this 'discrete lines' 
    % plotting technique.

    len= size(lne,2)*3 - 1;
    p= zeros(2, len);

    p(:,1:3:end)= lne(1:2,:);
    p(:,2:3:end)= lne(3:4,:);
    p(:,3:3:end)= NaN;
end

function p = make_covariance_ellipses(x,P)
    % Compute ellipses for plotting state covariances
    N = 10;
    inc = 2*pi/N;
    phi = 0:inc:2*pi;

    lenx = length(x);
    lenf = (lenx - 3)/2;
    p = zeros (2,(lenf + 1)*(N + 2));

    ii = 1:N + 2;
    p(:,ii) = make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

    ctr = N + 3;
    for i = 1:lenf
        ii = ctr:(ctr + N + 1);
        jj = 2 + 2*i;
        jj = jj:jj+1;

        p(:,ii) = make_ellipse(x(jj), P(jj,jj), 2, phi);
        ctr = ctr + N + 2;
    end
end

function p = make_ellipse(x, P, s, phi)
    % make a single 2-D ellipse of s-sigmas over phi angle intervals 
    r = sqrtm(P);
    a = s*r*[cos(phi); sin(phi)];
    p(2,:) = [a(2,:) + x(2) NaN];
    p(1,:) = [a(1,:) + x(1) NaN];
end

function h = ellipse(ra, rb, ang, x0, y0, C, Nb)
    % Ellipse adds ellipses to the current plot
    %
    % ELLIPSE(ra, rb, ang, x0, y0) adds an ellipse with semimajor axis of ra,
    % a semimajor axis of radius rb, a semimajor axis of ang, centered at
    % the point x0,y0.
    %
    % The length of ra, rb, and ang should be the same. 
    % If ra is a vector of length L and x0,y0 scalars, L ellipses
    % are added at point x0,y0.
    % If ra is a scalar and x0,y0 vectors of length M, M ellipse are with the same 
    % radii are added at the points x0,y0.
    % If ra, x0, y0 are vectors of the same length L=M, M ellipses are added.
    % If ra is a vector of length L and x0, y0 are  vectors of length
    % M~=L, L*M ellipses are added, at each point x0,y0, L ellipses of radius ra.
    %
    % ELLIPSE(ra,rb,ang,x0,y0,C)
    % adds ellipses of color C. C may be a string ('r','b',...) or the RGB value. 
    % If no color is specified, it makes automatic use of the colors specified by 
    % the axes ColorOrder property. For several circles C may be a vector.
    %
    % ELLIPSE(ra,rb,ang,x0,y0,C,Nb), Nb specifies the number of points
    % used to draw the ellipse. The default value is 300. Nb may be used
    % for each ellipse individually.
    %
    % h=ELLIPSE(...) returns the handles to the ellipses.
    %
    % as a sample of how ellipse works, the following produces a red ellipse
    % tipped up at a 45 deg axis from the x axis
    % ellipse(1,2,pi/8,1,1,'r')
    %
    % note that if ra=rb, ELLIPSE plots a circle
    %

    % written by D.G. Long, Brigham Young University, based on the
    % CIRCLES.m original 
    % written by Peter Blattner, Institute of Microtechnology, University of 
    % Neuchatel, Switzerland, blattner@imt.unine.ch


    % Check the number of input arguments 

    if nargin<1
      ra=[];
    end
    if nargin<2
      rb=[];
    end
    if nargin<3
      ang=[];
    end

    %if nargin==1,
    %  error('Not enough arguments');
    %end;

    if nargin<5
      x0=[];
      y0=[];
    end

    if nargin<6
      C=[];
    end

    if nargin<7
      Nb=[];
    end

    % set up the default values

    if isempty(ra), ra = 1;end
    if isempty(rb), rb = 1;end
    if isempty(ang), ang = 0;end
    if isempty(x0), x0 = 0;end
    if isempty(y0), y0 = 0;end
    if isempty(Nb), Nb = 300;end
    if isempty(C), C = get(gca,'colororder');end

    % work on the variable sizes

    x0=x0(:);
    y0=y0(:);
    ra=ra(:);
    rb=rb(:);
    ang=ang(:);
    Nb=Nb(:);

    if isstr(C),C=C(:);end

    if length(ra)~=length(rb)
      error('length(ra)~=length(rb)');
    end
    if length(x0)~=length(y0)
      error('length(x0)~=length(y0)');
    end

    % how many inscribed elllipses are plotted

    if length(ra)~=length(x0)
      maxk=length(ra)*length(x0);
    else
      maxk=length(ra);
    end

    % drawing loop

    for k=1:maxk

      if length(x0)==1
        xpos=x0;
        ypos=y0;
        radm=ra(k);
        radn=rb(k);
        if length(ang)==1
          an=ang;
        else
          an=ang(k);
        end
      elseif length(ra)==1
        xpos=x0(k);
        ypos=y0(k);
        radm=ra;
        radn=rb;
        an=ang;
      elseif length(x0)==length(ra)
        xpos=x0(k);
        ypos=y0(k);
        radm=ra(k);
        radn=rb(k);
        an = ang(k)
      else
        rada = ra(fix((k-1)/size(x0,1))+1);
        radb = rb(fix((k-1)/size(x0,1))+1);
        an = ang(fix((k-1)/size(x0,1))+1);
        xpos = x0(rem(k-1,size(x0,1))+1);
        ypos = y0(rem(k-1,size(y0,1))+1);
      end

      co = cos(an);
      si = sin(an);
      the = linspace(0,2*pi,Nb(rem(k-1,size(Nb,1))+1,:)+1);
    %  x=radm*cos(the)*co-si*radn*sin(the)+xpos;
    %  y=radm*cos(the)*si+co*radn*sin(the)+ypos;
      h(k) = line(radm*cos(the)*co - si*radn*sin(the) + xpos, radm*cos(the)*si + co*radn*sin(the) + ypos);
      set(h(k), 'color', C(rem(k-1, size(C,1)) + 1,:));

    end
end