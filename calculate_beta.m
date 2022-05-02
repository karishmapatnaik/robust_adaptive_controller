% Simulates measurement of beta from sensor angles. The dynamics of beta
% are modelled as a first order system

function [beta, betadot] = calculate_beta(beta,betadot,ext_f,dt)
%     betadot = betadot + (ext_f - 1.5*betadot - 2*beta)*dt;
    betadot = betadot + (ext_f - 2.1*betadot - 20*beta)*dt; % correct one
%     betadot = betadot + (ext_f - 1*betadot - 100*beta)*dt; % test low damping
    beta = beta + betadot*dt;
end
    