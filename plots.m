figure(1)
subplot(3,1,1)
plot(tfs,omegadfs(:,1),'-','LineWIdth',2)
hold on
plot(tfs,omegafs(:,1)','--','LineWIdth',2)
% axis([0 tend -0.5 0.5])
set(gca,'FontSize',30)
subplot(3,1,2)
plot(tfs,omegadfs(:,2),'-','LineWIdth',2)
hold on
plot(tfs,omegafs(:,2)','--','LineWIdth',2)
% axis([0 tend -0.4 0.4])
ylabel('\Omega (rad/s)')
set(gca,'FontSize',30)
subplot(3,1,3)
plot(tfs,omegadfs(:,3),'-','LineWIdth',2)
hold on
plot(tfs,omegafs(:,3)','--','LineWIdth',2)
% axis([0 tend -0.1 0.2])
xlabel('Time (s)')
set(gca,'FontSize',30)

figure(2)
subplot(3,1,1)
hold on
plot(tfs,efs_att(:,1),'-','LineWIdth',2)
% axis([0 tend -0.025 0.015])
set(gca,'FontSize',30)
subplot(3,1,2)
hold on
plot(tfs,efs_att(:,2)','-','LineWIdth',2)
% axis([0 tend -0.06 0.01])
ylabel('e_R ')
set(gca,'FontSize',30)
subplot(3,1,3)
hold on
plot(tfs,efs_att(:,3),'-','LineWIdth',2)
% axis([0 tend -10*10^-3 4*10^-3])
xlabel('Time (s)')
set(gca,'FontSize',30)

figure(3)
subplot(2,1,1)
plot(tfs,Jhatfs{1}(:,1),'LineWIdth',2)
hold on
plot(tfs,Jhatfs{1}(:,2),'LineWIdth',2)
plot(tfs,Jhatfs{1}(:,3),'LineWIdth',2)
plot(tfs,Jfs{1}(:,1),'--','LineWIdth',2)
plot(tfs,Jfs{1}(:,2),'--','LineWIdth',2)
plot(tfs,Jfs{1}(:,3),'--','LineWIdth',2)
plot(tfs,Jhatfs{2}(:,1),'LineWIdth',2)
plot(tfs,Jhatfs{2}(:,2),'LineWIdth',2)
plot(tfs,Jhatfs{2}(:,3),'LineWIdth',2)
plot(tfs,Jfs{2}(:,1),'--','LineWIdth',2)
plot(tfs,Jfs{2}(:,2),'--','LineWIdth',2)
plot(tfs,Jfs{2}(:,3),'--','LineWIdth',2)
% axis([0 tend 0 0.05])
ylabel ('h_{xx},h_{yy},h{zz}')
set(gca,'FontSize',30)
subplot(2,1,2)
plot(tfs,Jhatfs{1}(:,4),'LineWIdth',2)
hold on
plot(tfs,Jhatfs{1}(:,5),'LineWIdth',2)
plot(tfs,Jhatfs{1}(:,end),'LineWIdth',2)
plot(tfs,Jfs{1}(:,4),'--','LineWIdth',2)
plot(tfs,Jfs{1}(:,5),'--','LineWIdth',2)
plot(tfs,Jfs{1}(:,end),'--','LineWIdth',2)
plot(tfs,Jhatfs{2}(:,4),'LineWIdth',2)
plot(tfs,Jhatfs{2}(:,5),'LineWIdth',2)
plot(tfs,Jhatfs{2}(:,end),'LineWIdth',2)
plot(tfs,Jfs{2}(:,4),'--','LineWIdth',2)
plot(tfs,Jfs{2}(:,5),'--','LineWIdth',2)
plot(tfs,Jfs{2}(:,end),'--','LineWIdth',2)
% axis([0 tend -25*10^-3 10*10^-3])
xlabel('Time (s)')
ylabel ('h_{xy},h_{yz},h{xz}')
set(gca,'FontSize',30)

figure(4)
subplot(3,1,1)
hold on
plot(tfs,controlfs(:,1),'LineWIdth',2)
% axis([0 tend -0.02 0.02])
set(gca,'FontSize',30)
subplot(3,1,2)
hold on
plot(tfs,controlfs(:,2),'LineWIdth',2)
% axis([0 tend -0.02 0.02])
ylabel ('\tau')
set(gca,'FontSize',30)
subplot(3,1,3)
hold on
plot(tfs,controlfs(:,3),'LineWIdth',2)
% axis([0 tend -0.006 0.006])
xlabel('Time (s)')
set(gca,'FontSize',30)



