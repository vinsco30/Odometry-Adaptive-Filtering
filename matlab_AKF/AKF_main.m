clear 
close all

%Load plot_odom_ros.m 
plot_odom_ros
%Compute statistics
test_analyzer
%% KF creation
Dt = 1e-1;
tau1 = 1e-2;
tau2 = 1e-2;
tau3 = 1e-2;
a = [exp(-Dt/tau1); exp(-Dt/tau2); exp(-Dt/tau3)];

A = [eye(3), Dt*eye(3), Dt^2/2*eye(3), zeros(3,12);
    zeros(3), eye(3), Dt*eye(3), zeros(3,12);
    zeros(3), zeros(3), a.*eye(3), zeros(3,12);
    zeros(12,3), zeros(12,3), zeros(12,3), eye(12)];

B = [zeros(6,3); (1-a).*eye(3,3); zeros(12,3)];

H_L = [eye(3), zeros(3), zeros(3), -eye(3), zeros(3), zeros(3,6);
    zeros(3), eye(3), zeros(3), zeros(3), eye(3), zeros(3,6)];

H_V = [eye(3), zeros(3), zeros(3), zeros(3,6), -eye(3), zeros(3);
    zeros(3), eye(3), zeros(3), zeros(3,6), zeros(3), eye(3)];

qr = [1, 1, 1, 1, 1, 1, 1, 1, 1]*0.1;
qd = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]*0.1;
Q = zeros(21,21,size(ax_ref_def,1));
% Q = zeros(21,21);
Q(:,:,1) = [qr.*eye(9), zeros(9,12); zeros(12,9), qd.*eye(12)];

r_l = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]*0.1;
r_v = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]*0.1;

R_L = r_l.*eye(6);
R_V = r_v.*eye(6);

%% Test with Gazebo datasets

u = [ax_ref_def'; ay_ref_def'; az_ref_def'];
z_l = [X'; Y'; Z'; vx'; vy'; vz'];
z_v = [x_gt_def'; y_gt_def'; z_gt_def'; vx_gt_def'; vy_gt_def'; vz_gt_def'];
t = size(u,2);
x_p = zeros(21,t+1);
x_p(3,1) = z_gt_def(1,1);
x_s = zeros(21,t+1);
x_s(:,1) = x_p(:,1);
in=zeros(21,t);
P_p = zeros(21,21,t+1);
P_p(:,:,1) = 0.01*eye(21);
P_s = zeros(21,21,t+1);
P_s(:,:,1) = 0.01*eye(21);
y = zeros(6,t+1);
y(:,1) = z_v(:,1) - H_V*x_p(:,1);
K = zeros(21,6,t+1);
K(:,:,1) = P_p(:,:,1)*H_V'*inv(H_V*P_p(:,:,1)*H_V' + R_V);

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
q_vis_meas_lx_ok = 1e2;
q_vis_meas_ly_ok = 1e2;
q_vis_meas_lz_ok = 1e2;
r_lx_bad = 1e6;
r_ly_bad = 1e6;
r_lz_bad = 1e6;
q_lx_meas_bad = 1e8;
q_ly_meas_bad = 1e8;
q_lz_meas_bad = 1e8;
q_vx_meas_ok = 1e-7;
r_vx_meas_ok = 1e-7;
update_x = false;
update_y = false;
update_z = false;

err = zeros(3,t);
err_v = zeros(3,t);

for i=2:size(u,2)
    %Measurament vector building
    % z_l = [X(i); Y(i); Z(i); vx(i); vy(i-1); vz(i-1)];
    % z_v = [x_gt_def(i-1); y_gt_def(i-1); z_gt_def(i-1); vx_gt_def(i-1); vy_gt_def(i-1); vz_gt_def(i-1)];

    % Predict
    if meas_x_ok
        qdv_new = [qd(1,1:6), qd(1,7)*q_vis_meas_lx_ok, qd(1,8:9), ...
            qd(1,10)*q_vis_meas_lx_ok*1, qd(1,11:12)];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end
    if meas_y_ok
        qdv_new = [qd(1,1:6), qd(1,7), qd(1,8)*q_vis_meas_ly_ok, qd(1,9), ...
            qd(1,10), qd(1,11)*q_vis_meas_ly_ok*1, qd(1,12)];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end
    if meas_z_ok
        qdv_new = [qd(1,1:6), qd(1,7:8), qd(1,9)*q_vis_meas_lz_ok, ...
            qd(1,10:11), qd(1,12)*q_vis_meas_lz_ok*1];
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    end

    x_p(:,i) = A*x_s(:,i-1) + B*u(:,i-1);
    P_p(:,:,i) = A*P_s(:,:,i-1)*A' + Dt*Q(:,:,i-1);

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
        qdl_new = [qd(1,1), qd(1,2:3), qd(1,4)*q_lx_meas_bad, ...
            qd(1,5:end)];
        qdv_new = [qd(1,1:6), qd(1,7)*q_vx_meas_ok, qd(1,8:9), ...
            qd(1,10)*q_vx_meas_ok*0.1, qd(1,11:12)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
        % Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdv_new.*eye(12)];
    %     H_off_L = [zeros(3), zeros(3), zeros(3), -[1, 0, 0].*eye(3), zeros(3), zeros(3,6);
    % zeros(3), zeros(3), zeros(3), zeros(3), -[1, 0, 0].*eye(3), zeros(3,6)];
        update_x = true;
    end
    if ~meas_y_ok
        r_l_new = [r_l(1), r_l(2)*r_ly_bad, r_l(1,3), ...
            r_l(4), r_l(5)*r_ly_bad, r_l(6)];
        qdl_new = [qd(1,1), qd(1,2)*q_ly_meas_bad, qd(1,3), qd(1,4), ...
            qd(1,5)*q_ly_meas_bad, qd(1,6:end)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
        H_off_L = [zeros(3), zeros(3), zeros(3), -[0, 1, 0].*eye(3), zeros(3), zeros(3,6);
    zeros(3), zeros(3), zeros(3), zeros(3), -[0, 1, 0].*eye(3), zeros(3,6)];
        update_y=true;
    end
    if ~meas_z_ok
        r_l_new = [r_l(1,1:2), r_l(1,3)*r_lz_bad, r_l(1,4:5), r_l(6)*r_lz_bad];
        qdl_new = [qd(1,1:2), qd(1,3)*q_lz_meas_bad, qd(1,4:5), ...
            qd(1,6)*q_lz_meas_bad, qd(1,7:end)];
        R_L = r_l_new.*eye(6);
        Q(:,:,i) = [qr.*eye(9), zeros(9,12); zeros(12,9), qdl_new.*eye(12)];
    %     H_off_L = [zeros(3), zeros(3), zeros(3), -[0, 0, 1].*eye(3), zeros(3), zeros(3,6);
    % zeros(3), zeros(3), zeros(3), zeros(3), -[0, 0, 1].*eye(3), zeros(3,6)];
        update_z = true;
    end


    if mod(i,2) == 0 
        y(:,i) = z_v(:,i) - H_V*x_p(:,i);
        H = H_V;
        K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_V);
    elseif mod(i,2) == 1
        y(:,i) = z_l(:,i) - H_L*x_p(:,i);
        H = H_L;
        K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_L);
    end
    % K(:,:,i) = P_p(:,:,i)*H'*inv(H*P_p(:,:,i)*H' + R_L);
    x_s(:,i) = x_p(:,i) + K(:,:,i)*y(:,i);
    P_s(:,:,i) = (eye(21)-K(:,:,i)*H)*P_p(:,:,i);
    
    err(1,i) = abs(x_s(1,i)-x_gt_def(i));
    err(2,i) = abs(x_s(2,i)-y_gt_def(i));
    err(3,i) = abs(x_s(3,i)-z_gt_def(i));

    err_v(1,i) = abs(x_s(4,i)-vx_gt_def(i));
    err_v(2,i) = abs(x_s(5,i)-vy_gt_def(i));
    err_v(3,i) = abs(x_s(6,i)-vz_gt_def(i));

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
plot(t1,x_s(7,1:end-1),'Color','[0.07,0.62,1.00]',LineWidth=2')
hold on
plot(t1,ax_ref_def,':','Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$ax_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m/s^2]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,x_s(8,1:end-1),'Color','[1.00,0.41,0.16]',LineWidth=2)
hold on
% plot(t1,vy,'--','Color','[1.00,0.41,0.16]',LineWidth=2)
plot(t1,ay_ref_def,':','Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{sys}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m/s^2]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,x_s(9,1:end-1),'Color','[1.00,0.00,1.00]',LineWidth=2)
hold on
% plot(t1,vz,'--','Color','[1.00,0.00,1.00]',LineWidth=2)
plot(t1,az_ref_def,':','Color','[1.00,0.00,1.00]',LineWidth=2)
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

figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,err_v(1,1:end),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{x}$','interpreter','latex','Location','northeastoutside')
ylabel('${x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,err_v(2,1:end),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{y}$','interpreter','latex','Location','northeastoutside')
ylabel('${y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,err_v(1,1:end),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$err_{z}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${z}$ $[m]$','fontsize',18, 'interpreter','latex')

