% Main code to simulate all different controllers 

%% Store states
N = length(tfs);
omegafs = zeros(N,length(omega));
omegadfs = zeros(N,length(omega));
controlfs = zeros(N,3);
Jfs{1} = zeros(N,6);
Jfs{2} = zeros(N,6);
Jhatfs{1} = zeros(N,6);
Jhatfs{2} = zeros(N,6);
efs_att = zeros(N,7);
angfs = zeros(N,3);
angdfs = zeros(N,3);

%% Arm length profile
% uncomment this if the arm lengths change
lfs = 0.3*ones(length(tfs),1);
lfs(1:30000) = 0.3;
lfs(30000:60000) = 0.1;
lfs(60000:100000) = 0.3;
lfs(100000:150000) = 0.1;
l_arm_prev = lfs(1);

%% initialize J for each config
Jdot = zeros(3,3);
Jhat{1} = zeros(3,3);
Jhat{2} = zeros(3,3);
for i = 1:no_of_configs
    if i == 1
        l_arm = 0.3;
        cg = calculate_cg(beta,M_Sph,m_point,l_arm,h_prop);
        Jbarvec = current_J(beta,M_Sph,R_Sph,m_point,l_arm,h_prop);
        Jhat{i} = [Jbarvec(1) Jbarvec(4) Jbarvec(5);...
            Jbarvec(4) Jbarvec(2) Jbarvec(6);...
            Jbarvec(5) Jbarvec(6) Jbarvec(3)];
    elseif i == 2
        l_arm = 0.1;
        cg = calculate_cg(beta,M_Sph,m_point,l_arm,h_prop);
        Jbarvec = current_J(beta,M_Sph,R_Sph,m_point,l_arm,h_prop);
        Jhat{i} = [Jbarvec(1) Jbarvec(4) Jbarvec(5);...
            Jbarvec(4) Jbarvec(2) Jbarvec(6);...
            Jbarvec(5) Jbarvec(6) Jbarvec(3)]; 
    end
end
Phat{1} = (0.5*trace(Jhat{1})*eye(3)) - (Jhat{1});
Phat{2} = (0.5*trace(Jhat{2})*eye(3)) - (Jhat{2});

%% reinitialize counter and arm length for starting simulation
i = 1;
l_arm = 0.3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% main code begins %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% main code begins %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% main code
while t < tend-dt 
    Rd = eul2rotm([0 pi*cos(t)/9 pi*sin(t)/9]);

% calculate error in R
    eRvee = 0.5*(G*Rd'*R-R'*Rd*G);
    eR = [eRvee(3,2) eRvee(1,3) eRvee(2,1)]';
% calculate error in omega
    if i == 1
        omegad = [0 0 0]';
        omegad_prev = omegad;
    else
        omegadvee = Rd'*((Rd-Rd_prev)/dt);
        omegad = [omegadvee(3,2) omegadvee(1,3) omegadvee(2,1)]';
    end
   
    % for omegadot
    if i > 2 
        omegadotd = (omegad - omegad_prev)/dt;
    else
        omegadotd = [0 0 0]';
    end
    eOmega = omega - R'*Rd*omegad;
      
% Get current arm length
    l_arm = lfs(i);
    
% Set configuration
    if abs(l_arm - l_arm_prev) > 0.05
        config = next_config
        next_config = previous_config;
        previous_config = config;
    end
    
% Calculate J from geometry 
    % get vector J
    cg = calculate_cg(beta,M_Sph,m_point,l_arm,h_prop);
    Jbarvec = current_J(beta,M_Sph,R_Sph,m_point,l_arm,h_prop);
    % organize into the symmetric matrix
    if config == 1
        J{1} = [Jbarvec(1) Jbarvec(4) Jbarvec(5);...
            Jbarvec(4) Jbarvec(2) Jbarvec(6);...
            Jbarvec(5) Jbarvec(6) Jbarvec(3)] + diag([0.1 0.1 0.2]);
    elseif config == 2
        J{2} = [Jbarvec(1) Jbarvec(4) Jbarvec(5);...
            Jbarvec(4) Jbarvec(2) Jbarvec(6);...
            Jbarvec(5) Jbarvec(6) Jbarvec(3)] + diag([0.01 0.01 0.02]);
    end

% Calculate alpha_D
    alpha_D = - omegahat(omega)*R'*Rd*omegad + R'*Rd*omegadotd;

% MOI Estimator   
    switch estimator
        case 'proposed_controller'
            eA = eOmega + 0.1*eR;
            [Phat{config}, Jhat{config}] = calculate_Jtilde(eA, omega, alpha_D, Phat{config}, dt);
        case 'proposed_robust'
            eA = eOmega + c2*Jhat{config}^-1*eR;
            [Phat{config}, Jhat{config}] = calculate_Jtilde(eA, omega, alpha_D, Phat{config}, dt);
        case 'lee_robust'
            Jhat{config} = Jhat{config};
    end
          
% Controller
    switch estimator
        case 'proposed_controller'
            tau = -kR*eR -komega*eOmega + cross(omega, Jhat{config}*omega) + Jhat{config}*alpha_D;
        case 'lee_robust'
            eA = eOmega + c2*Jhat{config}^-1*eR;
            mu = -delta_R^2*eA/(delta_R*norm(eA)+epsilon); 
            tau = -kR_lee*eR -komega_lee*eOmega + cross(omega, Jhat{config}*omega) + Jhat{config}*alpha_D + mu;
        case 'proposed_robust'
            mu = -delta_RAd^2*eA/(delta_RAd*norm(eA)+epsilon);
            tau = -kR*eR -komega*eOmega + cross(omega, Jhat{config}*omega) + Jhat{config}*alpha_D + mu;
    end

% State propagation 
    states = {omega, R};
    control = {tau};
    [omega, R] = dynamics(states,control,J{config},Jdot,Delta_R(i));
    
% Store full states
    angfs(i,:) = rotm2eul(R);
    angdfs(i,:) = rotm2eul(Rd);
    if abs(angfs(end,3)) > deg2rad(maxRoll) 
        angfs(end,3) = sign(angfs(end,3))*deg2rad(maxRoll);
        R = eul2rotm([angfs(end,1) angfs(end,2) angfs(end,3)]);
    end
    if abs(angfs(end,2)) > deg2rad(maxPitch)
        angfs(end,2) = sign(angfs(end,2))*deg2rad(maxPitch);
        R = eul2rotm([angfs(end,1) angfs(end,2) angfs(end,3)]);
    end 
    omegafs(i,:) = omega';
    omegadfs(i,:) = omegad';
    Jfs{1,config}(i,:) = [J{1,config}(1) J{1,config}(5) J{1,config}(9) J{1,config}(2) J{1,config}(3) J{1,config}(6)];
    Jhatfs{1,config}(i,:) = [Jhat{1,config}(1) Jhat{1,config}(5) Jhat{1,config}(9) Jhat{1,config}(2) Jhat{1,config}(3) Jhat{1,config}(6)];
    
% Update previous states
    Rd_prev = Rd;
    omegad_prev = omegad;
    Jactprev = J;
    Jhat_prev = Jhat;
    l_arm_prev = l_arm;
    
% Store control
     controlfs(i,:) = tau';
    % store attitude errors
    Psi = 0.5*trace(G*(eye(3) - Rd'*R));
    efs_att(i,:) = [eR' eOmega' Psi];
    
% Update time, counter
    t = t + dt;
    i = i +1;
end

%% plot all data
plots
