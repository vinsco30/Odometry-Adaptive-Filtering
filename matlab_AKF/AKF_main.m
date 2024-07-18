clear 
close all

%% KF creation
Dt = 0.01;
tau1 = 0.1;
tau2 = 0.1;
tau3 = 0.2;
a = [exp(-Dt/tau1); exp(-Dt/tau2); exp(-Dt/tau3)];

A = [eye(3), Dt*eye(3), Dt^2/2*eye(3), zeros(3,12);
    zeros(3), eye(3), Dt*eye(3), zeros(3,12);
    zeros(3), zeros(3), a.*eye(3), zeros(3,12);
    zeros(12,3), zeros(12,3), zeros(12,3), eye(12)];

B = [zeros(6,3); eye(3)-a.*eye(3,3); zeros(12,3)];

H_L = [eye(3), zeros(3), zeros(3), -eye(3), zeros(3), zeros(3,6);
    zeros(3), eye(3), zeros(3), zeros(3), -eye(3), zeros(3,6)];

H_V = [eye(3), zeros(3), zeros(3), zeros(3,6), -eye(3), zeros(3);
    zeros(3), eye(3), zeros(3), zeros(3,6), zeros(3), -eye(3)];

qr = [1, 1, 1, 1, 1, 1, 1, 1, 1];
qd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];

Q = [qr.*eye(9), zeros(9,12); zeros(12,9), qd.*eye(12)];

r_l = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01];
r_v = [0.01, 0.01, 0.01, 0.1, 0.1, 0.1];

R_L = r_l.*eye(6);
R_V = r_v.*eye(6);

%% Simulated Trajectory
% t=2;
% dt=1e-3;
% tt=0:dt:t-dt;
% T=0:dt:6*t-dt;
% p0_=[0 0 0.2];
% p1_ = [0 0 0.2];
% p2_ = [0 0 2];
% p3_= [1 0 2];
% p4_ = [1 2 2];
% p5_ = [1 2 2];
% 
% [p1,pd1,pdd1]=mtraj(@tpoly,p0_,p1_,tt);
% [p2,pd2,pdd2]=mtraj(@tpoly,p1_,p2_,tt);
% [p3,pd3,pdd3]=mtraj(@tpoly,p2_,p3_,tt);
% [p4,pd4,pdd4]=mtraj(@tpoly,p3_,p4_,tt);
% [p5,pd5,pdd5]=mtraj(@tpoly,p4_,p5_,tt);
% [p6,pd6,pdd6]=mtraj(@tpoly,p5_,p5_,tt);
% 
% pd=[p1;p2;p3;p4;p5;p6];
% d_pd=[pd1;pd2;pd3;pd4;pd5;pd6];
% dd_pd=[pdd1;pdd2;pdd3;pdd4;pdd5;pdd6];
% % figure(1)
% subplot(3,1,1); plot(pd); xlabel('Time'); ylabel('p');
% subplot(3,1,2); plot(d_pd); xlabel('Time'); ylabel('pd');
% subplot(3,1,3); plot(dd_pd); xlabel('Time'); ylabel('pdd');
% 
% %Add noise
% pdx_n = awgn(pd(:,1),40,'measured');
% pdy_n = awgn(pd(:,2),40,'measured');
% pdz_n = awgn(pd(:,3),40,'measured');
% d_pdx_n = awgn(d_pd(:,1),30,'measured');
% d_pdy_n = awgn(d_pd(:,3),30,'measured');
% d_pdz_n = awgn(d_pd(:,3),30,'measured');
% %Add outliers
% cntx=0;
% cnty=0;
% cntz=0;
% cntvx=0;
% cntvy=0;
% cntvz=0;
% for i=1:length(pdx_n)
%     cntx=cntx+1;
%     cnty=cnty+1;
%     cntz=cntz+1;
%     if cntx==50
%         cntx=0;
%         pdx_n(i) = pdx_n(i)+0.2;
%     end
%     cntvx=cntvx+1;
%     if cntvx==2000
%         cntvx=0;
%         d_pdx_n(i) = d_pdx_n(i)-0.01;
%     end
%     if cnty == 90
%         cnty=0;
%         pdy_n(i) = pdy_n(i) - 0.15;
%     end
%     cntvy=cntvy+1;
%     if cntvy == 4000
%         cntvy=0;
%         d_pdy_n(i) = d_pdy_n(i)+0.015;
%     end
%     if cntz == 70
%         cntz=0;
%         pdz_n(i) = pdz_n(i) + 0.1;
%     end
%     cntvz=cntvz+1;
%     if cntvz == 3000
%         cntvz=0;
%         d_pdz_n(i) = d_pdz_n(i)+0.05;
%     end
% end
% pd_n = [pdx_n, pdy_n, pdz_n];
% d_pd_n = [d_pdx_n, d_pdz_n, d_pdz_n];

%% Simulated model
% u = [dd_pd(:,1)'; dd_pd(:,2)'; dd_pd(:,3)'];
% t = size(u,2);
% x_p = zeros(21,t+1);
% x_p(3,1) = p0_(3);
% x_s = zeros(21,t+1);
% x_s(:,1) = x_p(:,1);
% in=zeros(21,t);
% P_p = zeros(21,21,t+1);
% P_p(:,:,1) = 0.01*eye(21);
% P_s = zeros(21,21,t+1);
% P_s(:,:,1) = 0.01*eye(21);
% y = zeros(6,t+1);
% z = [pd_n(:,1)'; pd_n(:,2)'; pd_n(:,3)'; d_pd_n(:,1)'; d_pd_n(:,2)'; d_pd_n(:,3)'];
% y(:,1) = z(:,1) - H_L*x_p(:,1);
% K = zeros(21,6,t+1);
% K(:,:,1) = P_p(:,:,1)*H_L'*inv(H_L*P_p(:,:,1)*H_L' + R_L);
% for i=2:size(u,2)
%     %Predict
%     x_p(:,i) = A*x_s(:,i-1) + B*u(:,i-1);
%     P_p(:,:,i) = A*P_s(:,:,i)*A' + Dt*Q;
% 
%     %Update
%     y(:,i) = z(:,i) - H_L*x_p(:,i);
%     K(:,:,i) = P_p(:,:,i)*H_L'*inv(H_L*P_p(:,:,i)*H_L' + R_L);
%     x_s(:,i) = x_p(:,i) + K(:,:,i)*y(:,i);
%     P_s(:,:,i) = (eye(21)-K(:,:,i)*H_L)*P_p(:,:,i);
% end
% 
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(x_s(1,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
% ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(x_s(2,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
% ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(x_s(3,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

%% Test with Gazebo datasets
%Load plot_odom_ros.m 
plot_odom_ros
%Compute statistics
test_analyzer

u = [ax_ref_def'; ay_ref_def'; az_ref_def'];
z_l = [X'; Y'; Z'; vx'; vy'; vz'];
z_v = [x_gt_def'; y_gt_def'; z_gt_def'; vx_gt_def'; vy_gt_def'; vz_gt_def'];
t = size(u,2);
x_p = zeros(21,t+1);
x_p(3,1) = Z(1,1);
x_s = zeros(21,t+1);
x_s(:,1) = x_p(:,1);
in=zeros(21,t);
P_p = zeros(21,21,t+1);
P_p(:,:,1) = 0.01*eye(21);
P_s = zeros(21,21,t+1);
P_s(:,:,1) = 0.01*eye(21);
y = zeros(6,t+1);
y(:,1) = z_l(:,1) - H_L*x_p(:,1);
K = zeros(21,6,t+1);
K(:,:,1) = P_p(:,:,1)*H_L'*inv(H_L*P_p(:,:,1)*H_L' + R_L);

eig_xyz = [eig_x_def'; eig_y_def'; eig_z_def'];

%thresholds
k1_x = avg_x; k2_x = avg_x+1.0;
k1_y = avg_x; k2_y = avg_y+2.0;
k1_z = avg_x; k2_z = avg_z+2.0;

k_points = avg_points;
k_trace = avg_trace;

meas_x_ok = false;
meas_y_ok = false;
meas_z_ok = false;

meas_x_ok_norm = false;
for i=2:size(u,2)
    %Predict
    x_p(:,i) = A*x_s(:,i-1) + B*u(:,i-1);
    P_p(:,:,i) = A*P_s(:,:,i)*A' + Dt*Q;

    %Check measurements hysteresis
    if ~meas_x_ok & eig_xyz(1,i) > k2_x
        meas_x_ok=true;
    elseif meas_x_ok & eig_xyz(1,i) < k1_x
        meas_x_ok=false;
        % disp('Misura degradata istante:');
        % i
    end
    %Check only thresholds
    % if  eig_xyz(1,i) > k2_x
    %     meas_x_ok_norm=true;
    % elseif eig_xyz(1,i) < k1_x
    %     meas_x_ok_norm =false;
    %     disp('Misura degradata istante:');
    %     i
    % end
    %Update
    y(:,i) = z_l(:,i) - H_L*x_p(:,i);
    K(:,:,i) = P_p(:,:,i)*H_L'*inv(H_L*P_p(:,:,i)*H_L' + R_L);
    x_s(:,i) = x_p(:,i) + K(:,:,i)*y(:,i);
    P_s(:,:,i) = (eye(21)-K(:,:,i)*H_L)*P_p(:,:,i);
end

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,x_s(1,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,x_s(2,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,x_s(3,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(x_new(1,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
% ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(y_new(1,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
% ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(z_new(1,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

%% Symbolic 

% p = sym('p', [3 1]);
% v = sym('v', [3 1]);
% a = sym('a', [3 1]);
% 
% dt_l = sym('dt_l',[6 1]);
% dt_v = sym('dt_v',[6 1]);
% 
% x = [p; v; a; dt_l; dt_v];
% 
% z_L = H_L*x;
% z_V = H_V*x;