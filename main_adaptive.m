clear 
clc
% close all

global e3 m e1 dt t

%% load parameters
parameters

%% Set disturbance, if simple adaptive no disturbance
disturbance = 1;
Delta_R = ones(length(tfs),3).*[0 0 0];

%% run simple adaptive controller
J_gain = 0.0001;
attitude_adaptive %% this has config CRFQ-like

