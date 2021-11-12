%% 
%% Kinematic simulation of a land-based mobile robot
clear all; clc; close all;
%% Simulation parameters
dt = 0.1; % Step size
ts = 1000; % Simulation time
t = 0:dt:ts; % Time span
%%Vehicle (mobile robot)parameters(physical)
a = 0.254; %radius of the wheel(fixed)
d = 0.456;  % distance between wheel frame to vehicle frame(along y-axis)
l = 1.099; 

%% Initial conditions
x0 = 0;
y0 = 0;
psi0 = 0;
eta0 = [x0;y0;psi0];
eta(:,1) = eta0;
for i = 1:length(t) %Loop starts here
    psi = eta(3,i); % current orientation in rad.
    J_psi = [cos(psi),-sin(psi),0;
    sin(psi), cos(psi),0;
    0,0,1]; % Jacobian matrix
%Inputs
    V1_x = -0.5;%left wheel angular velocity
    V1_y = -0.5;%right wheel angular velocity
    V2_x = 0;
    V2_y = 0;
    V3_x = -0.5;
    V3_y = -0.5;
    V4_x = 0;
    V4_y = 0;
    V = [V1_x;V1_y;V2_x;V2_y;V3_x;V3_y;V4_x; V4_y];
    sigma_1=(l/(4*d^2 + 4*l^2))
    sigma_2=(d/(4*d^2 + 4*l^2))
%%Wheel configuration matrix
    W = [1/4,0,1/4,0,1/4,0,1/4,0;
         0,1/4,0,1/4,0,1/4,0,1/4;
         -sigma_2,sigma_1,-sigma_2,-sigma_1,sigma_2,-sigma_1,sigma_2,sigma_1];
%Velocity input commands
    zeta(:,i) = W*V; % Vector of velocity input commands

% Time derivative of generalized coordinates
    eta_dot(:,i) = J_psi * zeta(:,i);
%% Position propagation using the Euler method
    eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); % state update
end % loop ends here
%% Plotting functions
figure
plot(t, eta(1,1:i),'r-');
hold on
plot(t, eta(2,1:i),'b--');
plot(t, eta(3,1:i),'m-.');
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',24)
xlabel('t,[s]');
ylabel('\eta,[units]');

%% Animation (mobile robot motion animation)
l = l; % length of the mobile robot
w = d; % width of the mobile robot
% Mobile robot coordinates
mr_co = [-l/2,l/2,l/2,-l/2,-l/2;
-w/2,-w/2,w/2,w/2,-w/2;];
figure
for i = 1:5:length(t) % animation starts here
psi = eta(3,i);
R_psi = [cos(psi),-sin(psi);
sin(psi), cos(psi);]; % rotation matrix
v_pos = R_psi*mr_co;
fill(v_pos(1,:)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
hold on, grid on; axis([-1 3 -1 3]), axis square
plot(eta(1,1:i),eta(2,1:i),'b-');
legend('MR','Path'), set(gca,'fontsize',24)
xlabel('x,[m]'); ylabel('y,[m]');
pause(0.1); hold off
end % animation ends here
