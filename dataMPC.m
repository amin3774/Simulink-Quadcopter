
%% Linear MPC setup for 12-state quadcopter
% Requires A, B, M and g from the quadcopter model.
% No MPC Toolbox or Optimization Toolbox required.

Ts = 0.02;   % Sampling time (s)
N  = 15;     % Prediction horizon (0.6 seconds)

nx = 12;
nu = 4;

%% 1. Discretize the continuous-time system (ZOH)

Aug = [A B; zeros(nu, nx+nu)];
Ed = expm(Aug*Ts);

Ad = Ed(1:nx, 1:nx);
Bd = Ed(1:nx, nx+1:end);

%% 2. Define MPC weights

% State order:
% [x y z vx vy vz phi theta psi p q r]

xScale = [ ...
    0.5; 0.5; 0.2; ...                % Position (m)
    1.0; 1.0; 0.2; ...                % Velocity (m/s)
    15*pi/180; 15*pi/180; 30*pi/180; % Angles (rad)
    1.0; 1.0; 1.0];                 % Angular rates (rad/s)

% Control input scales:
% [Thrust; Roll torque; Pitch torque; Yaw torque]

uScale = [ ...
    0.5;
    0.003;
    0.003;
    0.0006];

Q = diag(1./xScale.^2);
Rnorm = eye(nu);

%% 3. Construct prediction matrices

Sx = zeros(nx*N, nx);
Su = zeros(nx*N, nu*N);

Ap = eye(nx);

for i = 1:N
    rows = (i-1)*nx + (1:nx);
    Ap = Ad*Ap;
    Sx(rows,:) = Ap;
    for j = 1:i
        cols = (j-1)*nu + (1:nu);
        Su(rows,cols) = Ad^(i-j)*Bd;
    end
end

%% 4. Normalize control inputs

G = kron(eye(N), diag(uScale));

SuN = Su*G;


% Terminal cost based on discrete-time LQR
% Uses Control System Toolbox

Bnorm = Bd * diag(uScale);

[~, Pterminal] = dlqr(Ad, Bnorm, Q, Rnorm);

% State prediction weights
Qbar = kron(eye(N), Q);

% Apply terminal cost to the final predicted state
Qbar(end-nx+1:end, end-nx+1:end) = Pterminal;

% Input weights
Rbar = kron(eye(N), Rnorm);


%% 5. Calculate MPC optimization matrices

Hmpc = 2*(SuN'*Qbar*SuN + Rbar);
Fmpc = 2*SuN'*Qbar*Sx;

% Enforce numerical symmetry
Hmpc = (Hmpc + Hmpc')/2;

%% 6. Gradient-descent step size

alpha = 1/norm(Hmpc, inf);

%% 7. Hover thrust and input limits

U0 = [M*g; 0; 0; 0];

Umin = [ ...
    0;
   -0.01;
   -0.01;
   -0.002];

Umax = [ ...
    10;
    0.01;
    0.01;
    0.002];

%% 8. Display dimensions

fprintf('Prediction horizon: %.2f s\n', N*Ts);
fprintf('Hmpc: %d x %d\n', size(Hmpc));
fprintf('Fmpc: %d x %d\n', size(Fmpc));
fprintf('alpha: %.8g\n', alpha);
