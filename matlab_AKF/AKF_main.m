clear 
close all

%Load plot_odom_ros.m 
plot_odom_ros
%Compute statistics
test_analyzer
%% KF creation
Dt = 0.01;
tau1 = 0.1;
tau2 = 0.1;
tau3 = 0.1;
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
qd = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1];
Q = zeros(21,21,size(ax_ref_def,1));
Q(:,:,1) = [qr.*eye(9), zeros(9,12); zeros(12,9), qd.*eye(12)];

r_l = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
r_v = [1, 1, 1, 1, 1, 1];

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
k1_x = avg_x; k2_x = avg_x+2.0;
k1_y = avg_y; k2_y = avg_y+2.0;
k1_z = avg_z; k2_z = avg_z+2.0;

k_points = avg_points;
k_trace = avg_trace;

meas_x_ok = true;
meas_y_ok = true;
meas_z_ok = true;
q_vis_meas_lx_ok = 1;
q_vis_meas_ly_ok = 1;
q_vis_meas_lz_ok = 1;
r_lx_bad = 1;
r_ly_bad = 1;
r_lz_bad = 1;
q_lx_meas_bad = 1;
q_ly_meas_bad = 1;
q_lz_meas_bad = 1;

err = zeros(3,t);

for i=2:size(u,2)
    %Predict
    if meas_x_ok
        qdv_new = [qd(1,1:6), qd(1,7)*q_vis_meas_lx_ok, qd(1,8:9), ...
            qd(1,10)*q_vis_meas_lx_ok*0.01, qd(1,11:12)];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end
    if meas_y_ok
        qdv_new = [qd(1,1:6), qd(1,7), qd(1,8)*q_vis_meas_ly_ok, qd(1,9), ...
            qd(1,10), qd(1,11)*q_vis_meas_ly_ok*0.01, qd(1,12)];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end
    if meas_z_ok
        qdv_new = [qd(1,1:6), qd(1,7:8), qd(1,9)*q_vis_meas_lz_ok, ...
            qd(1,10:11), qd(1,12)*q_vis_meas_lz_ok*0.01];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end

    x_p(:,i) = A*x_s(:,i-1) + B*u(:,i-1);
    P_p(:,:,i) = A*P_s(:,:,i)*A' + Dt*Q(:,:,i);

    %Check measurements hysteresis
    if ~meas_x_ok & eig_xyz(1,i) > k2_x
        meas_x_ok=true;
    elseif meas_x_ok & eig_xyz(1,i) < k1_x
        meas_x_ok=false;
    end
    if ~meas_y_ok & eig_xyz(2,i) > k2_y
        meas_y_ok=true;
    elseif meas_y_ok & eig_xyz(2,i) < k1_y
        meas_y_ok=false;
    end
    if ~meas_z_ok & eig_xyz(3,i) > k2_z
        meas_z_ok=true;
    elseif meas_z_ok & eig_xyz(3,i) < k1_z
        meas_z_ok=false;
    end

    %Update
    if ~meas_x_ok
        r_l_new = [r_l(1)*r_lx_bad, r_l(1,2:3), r_l(4)*r_lx_bad, r_l(5:6)];
        qdl_new = [qd(1,1)*q_lx_meas_bad, qd(1,2:3), qd(1,4)*q_lx_meas_bad, ...
            qd(1,5:end)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
    end
    if ~meas_y_ok
        r_l_new = [r_l(1), r_l(2)*r_ly_bad, r_l(1,3), ...
            r_l(4), r_l(5)*r_ly_bad, r_l(6)];
        qdl_new = [qd(1,1), qd(1,2)*q_ly_meas_bad, qd(1,3), qd(1,4), ...
            qd(1,5)*q_ly_meas_bad, qd(1,6:end)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
    end
    if ~meas_z_ok
        r_l_new = [r_l(1,1:2), r_l(1,3)*r_lz_bad, r_l(1,4:5), r_l(6)*r_lz_bad];
        qdl_new = [qd(1,1:2), qd(1,3)*q_lz_meas_bad, qd(1,4:5), ...
            qd(1,6)*q_lz_meas_bad, qd(1,7:end)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
    end
    if mod(i,2) == 0 
        y(:,i) = z_l(:,i) - H_L*x_p(:,i);
        H = H_L;
        K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_L);
    elseif mod(i,2) == 1
        y(:,i) = z_v(:,i) - H_V*x_p(:,i);
        H = H_V;
        K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_V);
    end
    % K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_L);
    x_s(:,i) = x_p(:,i) + K(:,:,i)*y(:,i);
    P_s(:,:,i) = (eye(21)-K(:,:,i)*H_L)*P_p(:,:,i);
    
    err(1,i) = abs(x_s(1,i)-x_gt_def(i));
    err(2,i) = abs(x_s(2,i)-y_gt_def(i));
    err(3,i) = abs(x_s(3,i)-z_gt_def(i));

end

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,x_s(1,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2')
hold on
plot(t1,X,'--','Color','[0.07,0.62,1.00]',LineWidth=2)
plot(t1,x_gt_def,':','Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,x_s(2,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
hold on
plot(t1,Y,'--','Color','[1.00,0.41,0.16]',LineWidth=2)
plot(t1,y_gt_def,':','Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,x_s(3,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
hold on
plot(t1,Z,'--','Color','[1.00,0.00,1.00]',LineWidth=2)
plot(t1,z_gt_def,':','Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,x_s(4,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2')
hold on
plot(t1,vx,'--','Color','[0.07,0.62,1.00]',LineWidth=2)
plot(t1,vx_gt_def,':','Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,x_s(5,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
hold on
plot(t1,vy,'--','Color','[1.00,0.41,0.16]',LineWidth=2)
plot(t1,vy_gt_def,':','Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,x_s(6,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
hold on
plot(t1,vz,'--','Color','[1.00,0.00,1.00]',LineWidth=2)
plot(t1,vz_gt_def,':','Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{sys}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,err(1,1:end),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{x}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,err(2,1:end),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{y}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,err(1,1:end),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{z}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

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