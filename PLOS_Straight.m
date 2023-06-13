% Pure pursuit and LOS (PLOS) Algorithm
clc, clear, close all
% Pre-allocate memory
t = zeros(1,500);
psi = zeros(1,500);
u = zeros(1,500);
x = zeros(1,500);
y = zeros(1,500);

%% Parameters
%.. Angle Converting Parameters
r2d = 180 / pi ;          % Radian to Degree [-]
d2r = 1 / r2d ;           % Degree to Radian [-]

%.. Time Step Size
dt  =  0.1 ;               % Time Step Size [s]

%.. Time
t(1) = 0 ;                 % Simulation Time [s]

%.. Waypoint
Wi = [ 0, 0 ]' ;         % Initial Waypoint Position [m]
Wf = [ 500, 500 ]' ;     % Final Waypoint Position [m]

%.. Position and Velocity of UAV
x(1) =  100 ;               % Initial UAV X Position [m]
y(1) =  0 ;                 % Initial UAV Y Position [m]
psi(1) =  0 * d2r ;           % Initial UAV Heading Angle [rad]

p(:,1) = [ x(1), y(1) ]' ;   % UAV Position Initialization [m]
va = 20 ;                % UAV Velocity [m/s]

%.. Maximum Lateral Acceleration of UAV
Rmin =  50 ;                % UAV Minimum Turn Radius [m]
umax =  va^2 / Rmin ;       % UAV Maximum Lateral Acceleration [m]

%.. Design Parameters
k1  =  5 ;                 % k1
k2  =  0.2 ;               % k2

%% Path Following Algorithm
i =  0 ;                 % Time Index

while x(i+1) < Wf(1)
    i = i + 1 ;
    
    % Path Following Algorithm
    
    % Step 1
    % Orientation of vector from current UAV position to final waypoint, thetad
    theta_d =  atan2(Wf(2) - p(2,i), Wf(1) - p(1,i));
    
    % Step 2
    % Distance between initial waypoint and current UAV position, Ru
    Ru =  norm(Wi - p(:,i));    
    % Orientation of vector from initial waypoint to final waypoint, theta
    theta =  atan2(Wf(2) - Wi(2), Wf(1) - Wi(1));
    
    % Step 3
    % Orientation of vector from initial waypoint to current UAV position, thetau
    theta_u =  atan2(p(2,i) - Wi(2), p(1,i) - Wi(1));
    % Difference between theta and theatu, DEL_theta
    DEL_theta =   theta - theta_u;
    
    % Step 4
    % Distance from current UAV position to desired path, d
    d = Ru * sin(DEL_theta);
    
    % Step 5
    % Desired heading angle rate, psi_dot
    psi_dot = k1 * (theta_d - psi(i)) + k2*d;
    
    % Step6
    % Guidance command, u
    u(i) = va * psi_dot;
    % Limit u
    if u(i) > umax
        u(i) = umax;
    elseif u(i) < -umax
        u(i) = - umax;
    end
    %==============================================================================%
    
    %.. UAV Dynamics
    % Dynamic Model of UAV
    dx = va * cos( psi(i) ) ;
    dy = va * sin( psi(i) ) ;
    dpsi = u(i) / va ;
    
    % UAV State Update
    x(i+1) = x(i) + dx * dt ;
    y(i+1) = y(i) + dy * dt ;
    psi(i+1) = psi(i) + dpsi * dt ;
    
    % UAV Position Vector Update
    p(:,i+1) = [ x(i+1), y(i+1) ]' ;
    
    %.. Time Update
    t(i+1) = t(i) + dt ;
end

% Remove unneccssary space
t = t(1:i+1);
psi = psi(1:i+1);
u = u(1:i);
x = x(1:i+1);
y = y(1:i+1);

%% Result Plot
%.. Trajectory Plot
figure(1) ;
plot( [ Wi(1), Wf(1) ], [ Wi(2), Wf(2) ], 'r--', 'LineWidth', 2 ) ;
hold on, grid on, grid minor, axis equal
plot( x, y, 'b','LineWidth', 1.2 ) ;
hold on ;
xlabel('X (m)') ;
ylabel('Y (m)') ;
legend('Desired Path', 'UAV Trajectory', 'Location', 'southeast' ) ;
% axis([ 0 500 0 500 ]) ;
title(['PLOS, k1 = ' num2str(k1) '   k2 = ' num2str(k2)])
set(gca, 'FontSize', 18)

%.. Guidance Command
figure(2) ;
plot( t(1:end-1), u, 'LineWidth', 2 ) ;
xlabel('Time (s)') ;
ylabel('u (m/s^2)') ;