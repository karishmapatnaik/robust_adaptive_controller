clear 
clc
% close all

global e3 m e1 dt t

%% load parameters
parameters

%% choose controller between 1, 2 or 3
desired =  1;
switch desired
    case 1
        estimator = 'proposed_controller';
        Delta_R = ones(length(tfs),3).*[0 0 0];
    case 2
        estimator = 'lee_robust';
        Delta_R = [0.1*sin(tfs); 0.1*sin(tfs); 0.01*sin(tfs)]';
    case 3
        estimator = 'proposed_robust';
        Delta_R = [0.1*sin(tfs); 0.1*sin(tfs); 0.01*sin(tfs)]';
end      

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

