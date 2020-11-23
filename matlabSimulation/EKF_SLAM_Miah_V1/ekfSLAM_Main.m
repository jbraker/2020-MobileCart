function ekfSLAM_Main

close all
clear all
clc

% % INPUTS from operating environment:
%     lm - set of s landmars (fixed)
%     wp - set of N waypoints that the vehicle is supposed to go through. 
%     dimention: 2 x N
%     
% % Vehicle INPUTS:
%     nu - linear speed  [m/s]
%     g  - read as gamma => steering angle [rad] (NOT steering speed)

% % OUTPUTS:
%     vehicle's estimated poses as it travel
%     landmarks' estimated poses

wp =[0, 9.5, 9.5, -10;
     0, 8, 19, 29];

lm = [-30, 10, 28, 30, 10, -20, -10;
      0.5, -20,-0.5, 19, 41, 39, 9]; 

% initialise states
qTrue= zeros(3,1);
%q= zeros(3,1); % vehicle actual state
%P= zeros(3);  % state co-variance matrix 

q= [0.1;0.1;0.05]; % vehicle actual state [m,m,rad]
P= diag((qTrue-q).^2);  % state co-variance matrix 

% initialise other variables and constants
T= 0.25; % [sec], sampling time interval between control signals
dtsum= 0; % change in time since last observation
ftag= 1:size(lm,2); % identifier for each landmark
da_table= zeros(1,size(lm,2)); % data association table 
iwp= 1; % index to first waypoint 
G= 0; % initial steer angle  % control input

% control parameters
V= 3; % m/s
MAXG= 30*pi/180; % [rad], maximum steering angle (-MAXG < g < MAXG)
RATEG= 20*pi/180; % [rad/s], maximum rate of change in steer angle
wheelBase= 4; % metres, vehicle wheel-base

%data= initialise_store(q,P,q); % stored data for off-line
% use poseRobot, and poseDesired to store data
%poseRobot = [];
%poseDesired =[];

% process (control) noise
sigmaV= 0.3; % [m/s] linear speed standard deviation 
sigmaG= (3.0*pi/180); % [rad], steering angle standard deviation
Q= [sigmaV^2 0; 0 sigmaG^2];


% observation parameters
MAX_RANGE= 30.0; % metres
DT_OBSERVE= 8*T; % seconds, time interval between observations

% observation noises
sigmaR= 0.1; % metres
sigmaB= (1.0*pi/180); % radians
R= [sigmaR^2 0; 0 sigmaB^2];

QE= Q; RE= R; 

% data association innovation gates (Mahalanobis distances)
GATE_REJECT= 4.0; % maximum distance for association
GATE_AUGMENT= 25.0; % minimum distance for creation of new feature
% For 2-D observation:
%   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
%   - percent probability mass is: 1-sigma bounds 40%, 2-sigma 86%, 3-sigma 99%, 4-sigma 99.9%.

% waypoint proximity
AT_WAYPOINT= 1.5; % metres, distance to current waypoint at which robot switches to next waypoint
%NUMBER_LOOPS= 3; % number of loops through the waypoint list

% switches
SWITCH_CONTROL_NOISE= 1; % if 0, velocity and gamma are perfect
SWITCH_SENSOR_NOISE = 1; % if 0, measurements are perfect
%SWITCH_ASSOCIATION_KNOWN= 0; % if 1, associations are given, if 0, they are estimated using gates
SWITCH_BATCH_UPDATE= 1; % if 1, process scan in batch, if 0, process sequentially

% Initial pose covariance estimate
P_init = diag([(qTrue(1)-q(1))^2;(qTrue(2)-q(2))^2;(qTrue(3)-q(3))^2]);

% main loop 
i = 1;

QDT(:,i) = q(1:3);
QTrueDT(:,i) = qTrue;
Pcov(:,:,i) = P_init;

fig=figure;
%set(fig, 'name', 'EKF-SLAM Simulator')

vid = VideoWriter('OUT/trajectory.avi');
vid.Quality = 100;
vid.FrameRate = 5; 
open(vid);

while iwp ~= 0
    clf;
    plot(lm(1,:),lm(2,:),'b*')
    hold on, axis equal

    plot(wp(1,:),wp(2,:), 'gd-')
    xlabel('X [m]'), ylabel('Y [m]')
    
    i=i+1;
    % compute true data
    [G,iwp]= compute_steering(qTrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, T);
    %%%%%%%%%%%%%%%%%%%%%       
    iwp
%     if (iwp==0 && NUMBER_LOOPS > 1) % perform loops: if final waypoint reached, go back to first
%         iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; 
%     end 
    
    if (iwp==0)
        iwp = 1;
    end
    
    qTrue= vehicle_model(qTrue, V,G, wheelBase,T);
    
    % add process noise
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);  % control input
   
    % EKF predict step
    [q,P]= predict (q,P, Vn,Gn,QE, wheelBase,T);
    
    %%%% if heading known, observe heading
    %%%% [q,P]= observe_heading(q,P, qTrue(3), SWITCH_HEADING_KNOWN);
        
    % EKF update step
    dtsum= dtsum + T;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        [z,ftag_visible]= get_observationsSuruz(qTrue, lm, ftag, MAX_RANGE);  % tag ID (ftag) is necessary if data association is known.
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
    
        %if SWITCH_ASSOCIATION_KNOWN == 1
        %    [zf,idf,zn, da_table]= data_associate_known(q,z,ftag_visible, da_table);
        %else %SWITCH_ASSOCIATION_KNOWN == 0 (associations are estimated using gates)
        [zf,idf, zn]= data_associate(q,P,z,RE, GATE_REJECT, GATE_AUGMENT);               
        %end
        % zf = feature measurements with data association getReject<4
        % zn = new feature measurements to be included.  distance> 25 [m]
        % Gate Augment

%         if SWITCH_USE_IEKF == 1  % we are not using Iterated EKF
%             [q,P]= update_iekf(q,P,zf,RE,idf, 5);
%         else   % use normal EKF 
          [q,P]= update(q,P,zf,RE,idf, SWITCH_BATCH_UPDATE); % normal EKF
%         end
          [q,P]= augment(q,P, zn,RE); 
    end
    
    % offline data store
%    data= store_data(data, q, P, qTrue);
    
    % Animation
    
    [Xa,Ya] = plot_DDMR((q(1:3))',axis(gca)); % 
    hold on;
    [Xd,Yd] = plot_DDMR((qTrue(1:3))',axis(gca)); % 
    hold on
    QDT(:,i) = q(1:3);
    QTrueDT(:,i) = qTrue;
    actual = plot(QDT(1,1:i),QDT(2,1:i),'r--');
    
    desired = plot(QTrueDT(1,1:i),QTrueDT(2,1:i),'b-');
    
    landmarkEstimate=plot(q(4:2:end), q(5:2:end),'r.','MarkerSize',16);
    
    
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
    
    if (i>2e3)  % time = DT_CONTROLS*2e3
        break;        
    end
    
    %hold off
end

close(vid);
tmp = 1;

function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%function [G,iwp]= compute_steering(x, wp, iwp, minD, G, rateG, maxG, dt)
%
% INPUTS:
%   x - true position
%   wp - waypoints
%   iwp - index to current waypoint
%   minD - minimum distance to current waypoint before switching to next
%   G - current steering angle
%   rateG - max steering rate (rad/s)
%   maxG - max steering angle (rad)
%   dt - timestep
%
% OUTPUTS:
%   G - new current steering angle
%   iwp - new current waypoint
%
% determine if current waypoint reached

cwp= wp(:,iwp);
d2= (cwp(1)-x(1))^2 + (cwp(2)-x(2))^2;
if d2 < minD^2
    iwp= iwp+1; % switch to next
    if iwp > size(wp,2) % reached final waypoint, flag and return
        iwp=0;
        return;
    end    
    cwp= wp(:,iwp); % next waypoint
end
% compute change in G (deltaG) to point towards current waypoint
% deltaG = desired angle - robot's actual heading angle - robot's steering angle
deltaG= pi_to_pi(atan2(cwp(2)-x(2), cwp(1)-x(1)) - x(3) - G);  

% limit rate
maxDelta= rateG*dt;
if abs(deltaG) > maxDelta
    deltaG= sign(deltaG)*maxDelta;
end

% limit angle
G= G+deltaG;
if abs(G) > maxG
    G= sign(G)*maxG;
end

function angle = pi_to_pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.
% Tim Bailey 2000

angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;

function xv= vehicle_model(xv, V,G, WB,T)
%
% INPUTS:
%   xv - vehicle pose [x;y;phi]
%   V - velocity
%   G - steer angle (gamma)
%   WB - wheelbase
%   T - change in time (discrete sampling time)
%
% OUTPUTS:
%   xv - new vehicle pose

xv= [xv(1) + V*T*cos(G+xv(3,:)); 
     xv(2) + V*T*sin(G+xv(3,:));
     pi_to_pi(xv(3) + V*T*sin(G)/WB)];

function [V,G]= add_control_noise(V,G,Q, addnoise)
%function [V,G]= add_control_noise(V,G,Q, addnoise)
%
% Add random noise to nominal control values. We assume Q is diagonal.

if addnoise == 1
    V= V + randn(1)*sqrt(Q(1,1));
    G= G + randn(1)*sqrt(Q(2,2));
end

function [x,P]= predict (x,P,v,g,Q,WB,T)
%function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance
%
% Tim Bailey 2004.

s= sin(g+x(3)); c= cos(g+x(3));
vts= v*T*s; vtc= v*T*c;

% jacobians   
Fk= [1 0 -vts;
     0 1  vtc;
     0 0 1];
Lk= [T*c -vts;
     T*s  vtc;
     T*sin(g)/WB v*T*cos(g)/WB];
  
% predict covariance
P(1:3,1:3)= Fk*P(1:3,1:3)*Fk' + Lk*Q*Lk';
if size(P,1)>3
    P(1:3,4:end)= Fk*P(1:3,4:end);
    P(4:end,1:3)= P(1:3,4:end)';
end    

% predict state
x(1:3)= [x(1) + vtc; 
         x(2) + vts;
         pi_to_pi(x(3)+ v*T*sin(g)/WB)];

function [X,Y] = plot_DDMR(Q,AX)
% PLOT_UNICYCLE   Function that generates lines for plotting a unicycle.
%
%    PLOT_UNICYCLE(Q,AX) takes in the axis AX = axis(gca) and the unicycle
%    configuration Q and outputs lines (X,Y) for use, for example, as:
%       fill(X,Y,'b')
%    This must be done in the parent script file.

x     = Q(1);
y     = Q(2);
theta = Q(3);

l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];

function [z,idf]= get_observationsSuruz(x, lm, idf, rmax)
%function [z,idf]= get_observationsSuruz(x, lm, idf, rmax)
%
% INPUTS:
%   x - vehicle pose [x;y;phi]
%   lm - set of all landmarks
%   idf - index tags for each landmark
%   rmax - maximum range of range-bearing sensor 
%
% OUTPUTS:
%   z - set of range-bearing observations
%   idf - landmark index tag for each observation
%
% Tim Bailey 2004.

[lm,idf]= get_visible_landmarks(x,lm,idf,rmax);
z= compute_range_bearing(x,lm);

function [lm,idf]= get_visible_landmarks(x,lm,idf,rmax)
% Select set of landmarks that are visible within vehicle's semi-circular field-of-view
dx= lm(1,:) - x(1);
dy= lm(2,:) - x(2);
phi= x(3);

% incremental tests for bounding semi-circle
%ii= find(abs(dx) < rmax & abs(dy) < rmax & (dx.^2 + dy.^2) < rmax^2);% bounding box and bounding circle
                 % 

% incremental tests for bounding semi-circle
ii= find(abs(dx) < rmax & abs(dy) < rmax ... % bounding box
      & (dx*cos(phi) + dy*sin(phi)) > 0 ...  % bounding line  x' = dx*cos(phi) + dy*sin(phi) => x coordinate of the landmark w.r.to robot coordinate system
      & (dx.^2 + dy.^2) < rmax^2);           % bounding circle
% Note: the bounding box test is unnecessary but illustrates a possible speedup technique
% as it quickly eliminates distant points. Ordering the landmark set would make this operation
% O(logN) rather that O(N).
  
lm= lm(:,ii);
idf= idf(ii);
%
function z= compute_range_bearing(x,lm)
% Compute exact observation
dx= lm(1,:) - x(1);
dy= lm(2,:) - x(2);
phi= x(3);
z= [sqrt(dx.^2 + dy.^2);   % [d1    d2    d3    ...
    atan2(dy,dx) - phi];   %  psi1  psi2  psi3  ...]

function z= add_observation_noise(z,R, addnoise)
%function z= add_observation_noise(z,R, addnoise)
%
% Add random measurement noise. We assume R is diagonal.    
if addnoise == 1
    len= size(z,2);
    if len > 0
        z(1,:)= z(1,:) + randn(1,len)*sqrt(R(1,1));
        z(2,:)= z(2,:) + randn(1,len)*sqrt(R(2,2));
    end
end

function [zf,idf, zn]= data_associate(x,P,z,R, gate1, gate2)
% 
% Simple gated nearest-neighbour data-association. No clever feature
% caching tricks to speed up association, so computation is O(N), where
% N is the number of features in the state.
%
% Tim Bailey 2004.

zf= []; zn= [];
idf= []; 

Nxv= 3; % number of vehicle pose states
Nf= (length(x) - Nxv)/2; % number of features already in map

% linear search for nearest-neighbour, no clever tricks (like a quick
% bounding-box threshold to remove distant features; or, better yet,
% a balanced k-d tree lookup). TODO: implement clever tricks.
for i=1:size(z,2)
    jbest= 0;
    nbest= inf;
    outer= inf;
    
    % search for neighbours
    for j=1:Nf
        [nis, nd]= compute_association(x,P,z(:,i),R, j);
        if nis < gate1 && nd < nbest % if within gate, store nearest-neighbour
            nbest= nd;
            jbest= j;
        elseif nis < outer % else store best nis value
            outer= nis;
        end
    end
    
    % add nearest-neighbour to association list
    if jbest ~= 0
        zf=  [zf  z(:,i)];
        idf= [idf jbest];
    elseif outer > gate2 % z too far to associate, but far enough to be a new feature
        zn= [zn z(:,i)];
    end
end

function [nis, nd]= compute_association(x,P,z,R,idf)
%
% return normalised innovation squared (ie, Mahalanobis distance) and normalised distance
[zp,H]= observe_model(x, idf);  % here idf is the index of a single feature
v= z-zp; 
v(2)= pi_to_pi(v(2));
S= H*P*H' + R;

nis= v'*inv(S)*v;
nd= nis + log(det(S));

function [z,H]= observe_model(x, idf)
%function [z,H]= observe_model(x, idf)
%
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

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state
H= zeros(2, length(x));

% auxiliary values
dx= x(fpos)  -x(1); 
dy= x(fpos+1)-x(2);
d2= dx^2 + dy^2;
d= sqrt(d2);
xd= dx/d;
yd= dy/d;
xd2= dx/d2;
yd2= dy/d2;

% predict z
z= [d;
    atan2(dy,dx) - x(3)];

% calculate H
H(:,1:3)        = [-xd -yd 0; yd2 -xd2 -1];
H(:,fpos:fpos+1)= [ xd  yd;  -yd2  xd2];


function [x,P]= update(x,P,z,R,idf, batch)
% function [x,P]= update(x,P,z,R,idf, batch)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   x, P - updated state and covariance

if batch == 1
    [x,P]= batch_update(x,P,z,R,idf);
else
    [x,P]= single_update(x,P,z,R,idf);
end

%
%

function [x,P]= batch_update(x,P,z,R,idf)

lenz= size(z,2);
lenx= length(x);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(x, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
[x,P]= KF_cholesky_update(x,P,v,RR,H);

%
%

function [x,P]= single_update(x,P,z,R,idf)

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(x, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    [x,P]= KF_cholesky_update(x,P,v,RR,H);
end

function [x,P]= KF_cholesky_update(x,P,v,R,H)
%function [x,P]= KF_cholesky_update(x,P,v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P]
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using Cholesky factorisation, which
% is more numerically stable than a naive implementation.
%
% Tim Bailey 2003
% Developed by Jose Guivant 

PHt= P*H';
S= H*PHt + R;

S= (S+S')*0.5; % make symmetric
SChol= chol(S);

SCholInv= inv(SChol); % triangular matrix
W1= PHt * SCholInv;
W= W1 * SCholInv';

x= x + W*v; % update 
P= P - W1*W1';

function [x,P]= augment(x,P,z,R) 
%function [x,P]= augment(x,P,z,R)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented state and covariance
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, as all measurements are assumed to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    [x,P]= add_one_z(x,P,z(:,i),R);
end

%
%

function [x,P]= add_one_z(x,P,z,R)

len= length(x);
r= z(1); b= z(2);
s= sin(x(3)+b); 
c= cos(x(3)+b);

% augment x
x= [x;
    x(1) + r*c;
    x(2) + r*s];

% jacobians
Gv= [1 0 -r*s;
     0 1  r*c];
Gz= [c -r*s;
     s  r*c];
     
% augment P
rng= len+1:len+2;
P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
P(rng,1:3)= Gv*P(1:3,1:3); % vehicle to feature xcorr
P(1:3,rng)= P(rng,1:3)';
if len>3
    rnm= 4:len;
    P(rng,rnm)= Gv*P(1:3,rnm); % map to feature xcorr
    P(rnm,rng)= P(rng,rnm)';
end


function p= make_laser_lines (rb,xv)
% compute set of line segments for laser range-bearing measurements
if isempty(rb), p=[]; return, end
len= size(rb,2);
lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3:4,:)= TransformToGlobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p= line_plot_conversion (lnes);

%
%

function p= make_covariance_ellipses(x,P)
% compute ellipses for plotting state covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;

lenx= length(x);
lenf= (lenx-3)/2;
p= zeros (2,(lenf+1)*(N+2));

ii=1:N+2;
p(:,ii)= make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

ctr= N+3;
for i=1:lenf
    ii= ctr:(ctr+N+1);
    jj= 2+2*i; jj= jj:jj+1;
    
    p(:,ii)= make_ellipse(x(jj), P(jj,jj), 2, phi);
    ctr= ctr+N+2;
end

%
%

function p= make_ellipse(x,P,s, phi)
% make a single 2-D ellipse of s-sigmas over phi angle intervals 
r= sqrtm(P);
a= s*r*[cos(phi); sin(phi)];
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

function p = TransformToGlobal(p, b)
% function p = TransformToGlobal(p, b)
%
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

function p= line_plot_conversion (lne)
%function p= line_plot_conversion (lne)
%
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

function h=ellipse(ra,rb,ang,x0,y0,C,Nb)
% Ellipse adds ellipses to the current plot
%
% ELLIPSE(ra,rb,ang,x0,y0) adds an ellipse with semimajor axis of ra,
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

if nargin<1,
  ra=[];
end;
if nargin<2,
  rb=[];
end;
if nargin<3,
  ang=[];
end;

%if nargin==1,
%  error('Not enough arguments');
%end;

if nargin<5,
  x0=[];
  y0=[];
end;
 
if nargin<6,
  C=[];
end

if nargin<7,
  Nb=[];
end

% set up the default values

if isempty(ra),ra=1;end;
if isempty(rb),rb=1;end;
if isempty(ang),ang=0;end;
if isempty(x0),x0=0;end;
if isempty(y0),y0=0;end;
if isempty(Nb),Nb=300;end;
if isempty(C),C=get(gca,'colororder');end;

% work on the variable sizes

x0=x0(:);
y0=y0(:);
ra=ra(:);
rb=rb(:);
ang=ang(:);
Nb=Nb(:);

if isstr(C),C=C(:);end;

if length(ra)~=length(rb),
  error('length(ra)~=length(rb)');
end;
if length(x0)~=length(y0),
  error('length(x0)~=length(y0)');
end;

% how many inscribed elllipses are plotted

if length(ra)~=length(x0)
  maxk=length(ra)*length(x0);
else
  maxk=length(ra);
end;

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
    end;
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
    an=ang(k)
  else
    rada=ra(fix((k-1)/size(x0,1))+1);
    radb=rb(fix((k-1)/size(x0,1))+1);
    an=ang(fix((k-1)/size(x0,1))+1);
    xpos=x0(rem(k-1,size(x0,1))+1);
    ypos=y0(rem(k-1,size(y0,1))+1);
  end;

  co=cos(an);
  si=sin(an);
  the=linspace(0,2*pi,Nb(rem(k-1,size(Nb,1))+1,:)+1);
%  x=radm*cos(the)*co-si*radn*sin(the)+xpos;
%  y=radm*cos(the)*si+co*radn*sin(the)+ypos;
  h(k)=line(radm*cos(the)*co-si*radn*sin(the)+xpos,radm*cos(the)*si+co*radn*sin(the)+ypos);
  set(h(k),'color',C(rem(k-1,size(C,1))+1,:));

end;
