clear 
clc
% close all

global e3 m e1 dt t

%% load parameters
parameters

%% Set disturbance, if simple adaptive no disturbance
disturbance = 1;
Delta_R = ones(length(tfs),3).*[0 0 0];

%% choose controller between 1, 2 or 3
desired =  2;
switch desired
    case 1
        estimator = 'proposed_robust_adaptive';
        J_gain = 0.0005; %% reduce gains from case with no disturbance
        if disturbance
            Delta_R = [0.001*sin(tfs); 0.001*sin(tfs); 0.001*sin(tfs)]';
        end
    case 2
        estimator = 'conventional_robust';
        if disturbance
            Delta_R = [0.001*sin(tfs); 0.001*sin(tfs); 0.001*sin(tfs)]';
        end        
end      

c2 = 0.05;
attitude %% this has config CRFQ-like

