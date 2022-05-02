clear 
clc
% close all
global e3 m e1 dt t

%% choose controller between 1, 2 or 3
desired =  2;
switch desired
    case 1
        estimator = 'proposed_controller';
    case 2
        estimator = 'lee_robust';
    case 3
        estimator = 'proposed_robust';
end      

%% load parameters
parameters

%% other simulation specs
Psi = [];
Rd_prev = eye(3);
omegad_prev = zeros(3,1);
angfs = rotm2eul(R);
maxRoll = 20;
maxPitch = 20;
angdfs = angfs;
e1 = [1 0 0]';

attitude %% this has config CRFQ-like

