
% Matlab code for inertia estimation and adaptive control

%% physical specs
beta = deg2rad([0;0]); % in rads
betadot = [0;0];
M_Sph = 1; % in kg
R_Sph = 5/100; % in m
m_point = 100/1000 ; % in kg
h_prop = -3/100; % in cm

%% simulation specs
dt = 0.001;
t = 0;
tend = 100; %32
tfs = linspace(0,tend+dt,tend/dt);
m = M_Sph + 4*m_point;
R = eye(3);
e3 = [0 0 1]';
omega = [0 0 0]';


%% controller gains
kR = 0.0424;
komega = 0.0296; 
% kR = 3.21;
% komega = 1.2;
G = diag([0.09 0.1 0.11]);
delta_RAd = 0.2;

%% configurations
no_of_configs = 2;
config = 1;
next_config = 2;
previous_config = 1;

%% robustness specs for Lee
% kR_lee = 3.21;
% komega_lee = 1.2;
kR_lee = 0.0424;
komega_lee = 0.0296;
delta_R = 0.2; % change this 0.5 for 
epsilon = 0.01;
c2 = 0.05;

%% other simulation specs
Psi = [];
Rd_prev = eye(3);
omegad_prev = zeros(3,1);
angfs = rotm2eul(R);
maxRoll = 20;
maxPitch = 20;
angdfs = angfs;
e1 = [1 0 0]';