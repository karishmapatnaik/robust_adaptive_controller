% Matlab function to calculate the rigid body dynamics after a given time 
% step with disturbance. The general quadcopter equations are solved using
% Euler integration

function [omega, R] = dynamics(states,control,J,Jdot)
global t
    dt = 0.001;
    omega = states{1};
    R = states{2};  
    tau = control{1};
    omega = omega + J^(-1)*(tau - cross(omega, J*omega) - Jdot*omega + 0.1*sin(t))*dt;
    R = R*expm(omegahat(omega)*dt);
    R = gramschmidt(R);
    

    
