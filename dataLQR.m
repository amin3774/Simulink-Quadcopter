%% Quadcopter Linearization and LQR Controller Design
% This script defines a 12-state nonlinear quadcopter model, linearizes
% its dynamics around the hovering equilibrium, and designs an LQR
% state-feedback controller. The weighting matrices Q and R are selected
% based on characteristic state deviations and control effort scales.
%
% State vector: [x y z vx vy vz phi theta psi p q r]'
% Control inputs: [Thrust RollTorque PitchTorque YawTorque]'
%
% Required MATLAB toolboxes:
%   1. Symbolic Math Toolbox   - Symbolic modeling and Jacobian calculation
%   2. Control System Toolbox  - LQR controller design (lqr function)
%
% Matrices A,B, and K saved in this folder are the result of linearization
% and LQR rates. If you are not able to run this script (lacking necessary
% toolboxes licence), simply use them. To run the Simulink model, you only
% need matrix K.

M = 0.5;                  %mass kg
l = 0.2;                  %distance between each motor and the center of mass
I =   [0.005 0     0;
       0     0.005 0;
       0     0     0.009];   % momentum of inertia kg*m2

Ix = 0.005;
Iy = 0.005;
Iz = 0.009;


g = 9.8;                   %gravity
km = 1e-7;                 %rotor reaction-torque coefficient Nm/(ras/s)^2
kf = 1e-5;

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
syms u1 u2 u3 u4
X = [x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12];
U = [u1;u2;u3;u4];

f = [x4;
    x5;
    x6;
    u1/M*(cos(x7)*sin(x8)*cos(x9) + sin(x7)*sin(x9));
    u1/M*(cos(x7)*sin(x8)*sin(x9) - sin(x7)*cos(x9));
    u1/M*cos(x7)*cos(x8)-g;
    x10 + x11*sin(x7)*tan(x8) + x12*cos(x7)*tan(x8);
    x11*cos(x7) - x12*sin(x7);
    x11*sin(x7)/cos(x8) + x12*cos(x7)/cos(x8);
    (u2 + (Iy - Iz)*x11*x12)/Ix;
    (u3 + (Iz - Ix)*x10*x12)/Iy;
    (u4 + (Ix - Iy)*x10*x11)/Iz];

J = jacobian(f, X);
x0 = zeros(12,1);
u0 = [M*g;0;0;0];
A = subs(J, X, x0);
A = double(subs(A, U, u0));

B = jacobian(f, U);
B = double(subs(B, X, x0));

% Characteristic state-deviation scales
xmax = [ ...
    2; 2; 1; ...                         % Positions (m)
    1; 1; 0.5; ...                       % Velocities (m/s)
    15*pi/180; 15*pi/180; 30*pi/180; ... % Angles (rad)
    1; 1; 0.5];                          % Angular rates (rad/s)

% Characteristic control-effort scales
umax = [ ...
    2; ...       % Thrust deviation (N)
    0.01; ...    % Roll torque (N.m)
    0.01; ...    % Pitch torque (N.m)
    0.002];      % Yaw torque (N.m)

Q = diag(1 ./ xmax.^2);
R = diag(1 ./ umax.^2);

[K,P,poles] = lqr(A,B,Q,R)
