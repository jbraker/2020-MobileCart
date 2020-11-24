clear all; close all; clc;

%% Initialization
% IP of the computer running CoppeliaSim so that it can be seen by the
% computer running Matlab. 
% Use '127.0.0.1' if Matlab and CoppeliaSim are NATIVELY running on the
% same computer.
computerIP = '127.0.0.1'; 
%%%%%%%%%%% Setup starts %%%%%%%%%%%%%%%%%%
% Make sure to add simRemoteApi.start(19999) in CoppeliaSim
% import remoteApi/helper functions in the libCoppeliaSim/ directory, for instance 
addpath('libCoppeliaSim/');  

%%%%%%%%% Start CoppeliaSim connection  %%%%%%%%%%%%%%%
[sim, clientID, error] = startConnection(computerIP);
if error==true
    return;
end

%%%%%%%%% Get handles  %%%%%%%%%%%%%%%
[robot_id, remote_id] = getHandles(sim, clientID);

%%%%%%%%%%%%% Setup ends here %%%%%%%%%%%%%%%%

% start the simulation:
% sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);


%%
% Stream robot position and orientation from CoppeliaSim
sim.simxGetObjectPosition(clientID, robot_id(1,1), -1, sim.simx_opmode_streaming); % Stream position of the robot
sim.simxGetObjectOrientation(clientID, robot_id(1,1), -1, sim.simx_opmode_streaming); % Stream orientation of the robot

%%
% Main control loop
%

%% Setup
T = 0.25; % [s], Sampling time interval between control signals

% Waypoints for the remote to follow
wp = [2.5, -2.5, -2.5,  2.5, 2.5;
      2.5,  2.5, -2.5, -2.5, 2.5];
% Remote trajectory
vr = 1; % [m/s] Remote velocity
dStep = 5/floor(5/(vr*T)); % Distance of each step
x1 = 2.5:-dStep:-2.5;
y1 = 2.5*ones(1, length(x1));
x2 = -2.5*ones(1, length(x1));
y2 = 2.5:-dStep:-2.5;
x3 = fliplr(x1);
y3 = -y1;
x4 = -x2;
y4 = fliplr(y2);
remTraj = [x1 x2 x3 x4; y1 y2 y3 y4];
% Initial actual position of remote
qrTrue = remTraj(:, 1);
numSteps = size(remTraj, 2);
% Height of remote
remHeight = 0.5; % [m]

% Landmarks: fixed RF sensor locations
lm = [-4,   -3, 1, 3.5, -1;
       3, -2.5, 2,  -4,  0];
  
% Initialize actual state of the robot
qTrue = getStates(sim, clientID, robot_id); % [m, m, rad]
% Initialize estimated state of the robot
q = qTrue + [0.1; 0.1; 0.05]; % [m, m, rad]

% Initialize state co-variance matrix
P = diag((qTrue - q).^2);

% Initialize other variables and constants
ftag = 1:size(lm, 2); % Identifier for each landmark
da_table = zeros(1, size(lm, 2)); % Data association table

% Control parameters
v = 1; % [m/s], Constant linear velocity
MAXW = 20*pi/180; % [rad], Maximum angular velocity (-MAXW < w < MAXW)
KW = 2; % Proportional gain for angular velocity

% Process (control) noise
sigmaV = 0.3; % [m/s], Linear speed standard deviation
sigmaW = (3.0*pi/180); % [rad], Steering angle standard deviation
Q = [sigmaV^2 0; 0 sigmaW^2]; % Process noise covariance matrix

% Observation parameters
MAX_RANGE = 30.0; % [m], Maximum sensing distance
DT_OBSERVE = 8*T; % [s], Time interval between observations
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
vid = VideoWriter('OUT/trajectory.avi');
vid.Quality = 100;
vid.FrameRate = 10; 
open(vid);

done = false;
k = 0;
while k < 2*numSteps % 2 times around
    % Clear the figure and plot the landmarks
    clf;
    hold on;
    axis([-5 5 -5 5]); pbaspect([1 1 1]);
    plot(lm(1, :), lm(2, :), 'b*');
    xlabel('X [m]'); ylabel('Y [m]');
    
    % Update remote position
    qrTrue = remTraj(:, mod(k-1, numSteps)+1);
    sim.simxSetObjectPosition(clientID, remote_id, -1, [qrTrue; remHeight], sim.simx_opmode_oneshot);
    
    % Advance timestep
    k = k + 1;
    
    %% Control inputs
    % Compute steering angle based on true pose
    w = compute_steering(qTrue, qr, KW, MAXW);
    qTrue = vehicle_model(qTrue, v, w, T);
    
    % Add process noise
    [vn, wn] = add_control_noise(v, w, Q, SWITCH_CONTROL_NOISE);
    
    %% EKF predict step
    [q, P] = predict(q, P, vn, wn, QE, T);
    
    %%%%%%%%%%%%% Measuring the states %%%%%%%%%%%%%%%%%%
    % This is where we measure all what will go inside
    % the state vector x
    
    qVector = getStates(sim, clientID, robot_id);
    x = qVector(1);
    y = qVector(2);
    theta = qVector(3);
    
    % Measure the distance to the remote
    [d_rem, theta_rem] = getRemotePosition(qVector, remPos);
    
%     disp(['d_rem: ' num2str(d_rem) ', theta_rem: ' num2str(theta_rem)]);
    
    % Target point w.r.t. robot's local coordinate frame
    [xRef, yRef] = pol2cart(theta_rem, d_rem - 1);
    % Find target point w.r.t. global coordinate frame
    R = [cos(theta) sin(theta); -sin(theta) cos(theta)];
    targ_global = R'*[xRef; yRef] + [x; y];
    sim.simxSetObjectPosition(clientID, target_id, -1, [targ_global; 0.002], sim.simx_opmode_oneshot);
    
%     disp(['xRef: ' num2str(xRef) ', yRef: ' num2str(yRef)]);

    sim.simxSetJointPosition(clientID, robot_id(4), k*pi/40, sim.simx_opmode_oneshot);
    

    %%%%%%%%%%%%% Computing the actuation %%%%%%%%%%%%%%%%%%
    % This is where we compute the actuation signal to
    % go into vector u
    
    [uVector, v(k), omega(k), e_x(k), e_y(k)] = controlAlgorithm(qVector, [xRef; yRef; theta_rem]);
    
    %%%%%%%%%%%%% Applying the actuation %%%%%%%%%%%%%%%%%%
    applyActuation(sim, clientID, robot_id, uVector);
    
    %%%%%%%%%%%%% Evaluate stopping criteria %%%%%%%%%%%%%%%%%%
    done = terminateSimulation(k, T, tf);
    
    pause(T); % move the robot for T second  
end


%%%% Final send command to stop the robot in CoppeliaSim 
nuR = 0.0; % [m/s]
nuL = 0.0; % [m/s]
applyActuation(sim, clientID, robot_id, [nuR; nuL]);


% stop the simulation:
% sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);

%%%%%%%%% Close CoppeliaSim connection  %%%%%%%%%%%%%%% 
closeConnection(sim, clientID);

%% Distance measurement function
function [d, theta] = getRemotePosition(q_rob, qp_rem)
    global sigm;
    
    % 3D position of robot
    qp_rob = [q_rob(1); q_rob(2); 0];
    
    % Calculate noisy distance measurement
    d = norm(qp_rem - qp_rob) + sigm*randn;
    theta = atan2(qp_rem(2) - qp_rob(2), qp_rem(1) - qp_rob(1)) - q_rob(3);
end

%% CoppeliaSim interface functions
function qVector = getStates(sim, clientID, robot_id)
    % Read the pose from CoppeliaSim
    [~, position] = sim.simxGetObjectPosition(clientID, robot_id(1), -1, sim.simx_opmode_buffer);
    [~, orientation] = sim.simxGetObjectOrientation(clientID, robot_id(1), -1, sim.simx_opmode_buffer);
    theta = orientation(3);
    qVector = [position(1),position(2),theta];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function applyActuation(sim, clientID, robot_id, uVector)
%%% Apply robot's actuator commands (right wheel and left wheel linear speeds) using the following APIs. %%%%%
    sim.simxPauseCommunication(clientID,1);
    % Right wheel
    sim.simxSetJointTargetVelocity(clientID,robot_id(2),uVector(1),sim.simx_opmode_oneshot); 
    % Left wheel
    sim.simxSetJointTargetVelocity(clientID,robot_id(3),uVector(2),sim.simx_opmode_oneshot); 
    sim.simxPauseCommunication(clientID,0);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function done = terminateSimulation(iter, T, tf)
%%% Evaluate the stopping criteria and update the boolean done
    done = false;
    if T*iter >= tf
        done = true;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [sim, clientID, error] = startConnection(computerIP)
%%%%%%%%% Start CoppeliaSim connection  %%%%%%%%%%%%%%%   

    error = false;
    disp('Program started');
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart(computerIP,19999,true,true,5000,5);

    if (clientID>-1)
        % enable the synchronous mode on the client:
        sim.simxSynchronous(clientID,true);
        
        disp('Connected to remote API server');
        
        % Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    else
        disp('Failed connecting to remote API server');
        sim.simxFinish(clientID); % close the connection to CoppeliaSim:
        sim.delete(); % call the destructor!
        error = true;
        return;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function closeConnection(sim, clientID)
%%%%%%%%% Close CoppeliaSim connection  %%%%%%%%%%%%%%%    

    % Before closing the connection to CoppeliaSim, make sure that the last command 
    % sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID);
    % Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID);
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [robot_id, remote_id] = getHandles(sim, clientID)
%%%%%%%%% Get handles  %%%%%%%%%%%%%%%
    % Get robot handles
    [rtn, robot_id(1)] = sim.simxGetObjectHandle(clientID, 'BudgetBot', sim.simx_opmode_oneshot_wait);
    if (rtn~=sim.simx_return_ok)
        fprintf('Get object handle failed\n');
    end
    [rtn, robot_id(2)]=sim.simxGetObjectHandle(clientID,'RightMotor', sim.simx_opmode_oneshot_wait);
    if (rtn~=sim.simx_return_ok)
        fprintf('Get right motor handle failed\n');
    end
    [rtn, robot_id(3)]=sim.simxGetObjectHandle(clientID,'LeftMotor', sim.simx_opmode_oneshot_wait);
    if (rtn~=sim.simx_return_ok)
        fprintf('Get left motor handle failed\n');
    end
%     [rtn, robot_id(4)]=sim.simxGetObjectHandle(clientID,'StepperMotor', sim.simx_opmode_oneshot_wait);
%     if (rtn~=sim.simx_return_ok)
%         fprintf('Get stepper motor handle failed\n');
%     end
    
    % Get remote handle
    [rtn, remote_id] = sim.simxGetObjectHandle(clientID, 'Remote', sim.simx_opmode_oneshot_wait);
    if (rtn~=sim.simx_return_ok)
        fprintf('Get remote handle failed\n');
    end
    
%     % Get target handle
%     [rtn, target_id] = sim.simxGetObjectHandle(clientID, 'TargetPoint', sim.simx_opmode_oneshot_wait);
%     if (rtn~=sim.simx_return_ok)
%         fprintf('Get target handle failed\n');
%     end
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