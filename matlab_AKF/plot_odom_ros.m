
close all

bag = rosbag("gazebo_dataset/point_lio_drift.bag");
bSel = select(bag,'Topic','/aft_mapped_to_init');
bSel1 = select(bag,'Topic','/uav1/hw_api/odometry');
bref = select(bag, 'Topic','/uav1/control_manager/control_reference');
bref1 = select(bag, 'Topic','/uav1/control_manager/estimator_input');
bMetrics = select(bag, 'Topic','/point_lio/eig');
bMetrics2 = select(bag, 'Topic','/point_lio/n_points');
bMetrics3 = select(bag, 'Topic','/point_lio/trace');

msgStructs = readMessages(bSel,'DataFormat','struct');
msgStructs1 = readMessages(bSel1,'DataFormat','struct');
msgRef = readMessages(bref,'DataFormat','struct');
msgRef1 = readMessages(bref1,'DataFormat','struct');

msgM = readMessages(bMetrics,'DataFormat','struct');
msgM2 = readMessages(bMetrics2,'DataFormat','struct');
msgM3 = readMessages(bMetrics3,'DataFormat','struct');

%Point LIO
X = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs);
Y = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs);
Z = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs);
vx = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs);
vy = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs);
vz = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs);
% Ground Truth
x_gt = cellfun(@(m) double(m.Pose.Pose.Position.X), msgStructs1);
y_gt = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgStructs1);
z_gt = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgStructs1);
vx_gt = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgStructs1);
vy_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgStructs1);
vz_gt = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgStructs1);

% Reference Trajectory
x_ref = cellfun(@(m) double(m.Pose.Pose.Position.X), msgRef);
y_ref = cellfun(@(m) double(m.Pose.Pose.Position.Y), msgRef);
z_ref = cellfun(@(m) double(m.Pose.Pose.Position.Z), msgRef);
vx_ref = cellfun(@(m) double(m.Twist.Twist.Linear.X), msgRef);
vy_ref = cellfun(@(m) double(m.Twist.Twist.Linear.Y), msgRef);
vz_ref = cellfun(@(m) double(m.Twist.Twist.Linear.Z), msgRef);
ax_ref = cellfun(@(m) double(m.ControlAcceleration.X), msgRef1);
ay_ref = cellfun(@(m) double(m.ControlAcceleration.Y), msgRef1);
az_ref = cellfun(@(m) double(m.ControlAcceleration.Z), msgRef1);

% Degradation metrics
eig_x = cellfun(@(m) double(m.Data(7,1)), msgM);
eig_y = cellfun(@(m) double(m.Data(8,1)), msgM);
eig_z = cellfun(@(m) double(m.Data(9,1)), msgM);
eig_velx = cellfun(@(m) double(m.Data(10,1)), msgM);
eig_vely = cellfun(@(m) double(m.Data(11,1)), msgM);
eig_velz = cellfun(@(m) double(m.Data(12,1)), msgM);
n_points = cellfun(@(m) double(m.Data), msgM2);
trace = cellfun(@(m) double(m.Data), msgM3);

timestamps1 = [];%timestamp point lio
timestamps2 = [];%timestamp acceleration input
timestamps3 = [];%timestamp ground truth
timestamps4 = [];%timestamp eigenvalues
t1 = [];
t2 = [];
t3 = [];
t4 = [];
sec = [];

for i=1:length(msgStructs)
    timestamps1 = [timestamps1; msgStructs{i,1}.Header.Stamp];
end
sec = [timestamps1.Sec];
nsec = [timestamps1.Nsec];
for i=1:length(sec)
    t1 = [t1; double(sec(i))+double(nsec(i))*1e-9];
end
for i=1:length(msgRef1)
    timestamps2 = [timestamps2; msgRef1{i,1}.Header.Stamp];
end
sec = [timestamps2.Sec];
nsec = [timestamps2.Nsec];
for i=1:length(sec)
    t2 = [t2; double(sec(i))+double(nsec(i))*1e-9];
end

for i=1:length(msgStructs1)
    timestamps3 = [timestamps3; msgStructs1{i,1}.Header.Stamp];
end
sec = [timestamps3.Sec];
nsec = [timestamps3.Nsec];
for i=1:length(sec)
    t3 = [t3; double(sec(i))+double(nsec(i))*1e-9];
end

% l=423;
% f=2500;


% Resampling acceleration inputs for AKF
rapp = size(t1,1)/size(t2,1);
fraction = sym(rapp);
[num, den] = numden(fraction);
l = double(num);
f = double(den);
ax_ref_def = resample(ax_ref,l,f);
ay_ref_def = resample(ay_ref,l,f);
az_ref_def = resample(az_ref,l,f);
% ax_ref_def = ax_ref_resampled(1:end,1);
% ay_ref_def = ay_ref_resampled(1:end,1);
% az_ref_def = az_ref_resampled(1:end,1);

%Resampling second source of odometry
rapp1 = size(t1,1)/size(t3,1);
fraction1 = sym(rapp1);
[num1, den1] = numden(fraction1);
ll = double(num1);
ff = double(den1);
x_gt_def = resample(x_gt,ll,ff);
y_gt_def = resample(y_gt,ll,ff);
z_gt_def = resample(z_gt,ll,ff);
vx_gt_def = resample(vx_gt,ll,ff);
vy_gt_def = resample(vy_gt,ll,ff);
vz_gt_def = resample(vz_gt,ll,ff);

%Resampling degradation metrics
rapp2 = size(t1,1)/size(eig_x,1);
fraction2 = sym(rapp2);
[num2, den2] = numden(fraction2);
lll = double(num2);
fff = double(den2);
eig_x_def = resample(eig_x,lll,fff);
eig_y_def = resample(eig_y,lll,fff);
eig_z_def = resample(eig_z,lll,fff);
eig_velx_def = resample(eig_velx,lll,fff);
eig_vely_def = resample(eig_vely,lll,fff);
eig_velz_def = resample(eig_velz,lll,fff);
trace_def = resample(trace,lll,fff);
n_points_def = resample(n_points,lll,fff);

%Discrete time derivative of the metrics
d_eigx = zeros(size(eig_x_def,1),1);
for k=2:size(eig_x_def,1)
    d_eigx(k,1) = (eig_x_def(k,1)-eig_x_def(k-1,1))/0.01;
end
%% Plot
% Together
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(ts1_resampled.Time, ts1_resampled.Data,'Color','[0.07,0.62,1.00]',LineWidth=2)
% hold on
% plot(ts2_resampled.Time, ts2_resampled.Data, 'Color','[0.07,0.62,1.00]','--',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$x_{fb}$','interpreter','latex','Location','northeastoutside')
% ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(Y((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$y_{fb}$','interpreter','latex','Location','northeastoutside')
% ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(Z((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$z_{fb}$','interpreter','latex','Location','northeastoutside')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('${p_z}$ $[m]$','fontsize',18, 'interpreter','latex')

%Plot Point LIO odom
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,X((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{fb}$','interpreter','latex','Location','northeastoutside')
ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,Y((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{fb}$','interpreter','latex','Location','northeastoutside')
ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,Z((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{fb}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${p_z}$ $[m]$','fontsize',18, 'interpreter','latex')
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,vx((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vx_{fb}$','interpreter','latex','Location','northeastoutside')
ylabel('${v_x}$ $[m/s]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,vy((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vy_{fb}$','interpreter','latex','Location','northeastoutside')
ylabel('${v_y}$ $[m/s]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,vz((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vz_{fb}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${v_z}$ $[m/s]$','fontsize',18, 'interpreter','latex')
% 
% %Plot GT
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,x_gt_def((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$x_{gt}$','interpreter','latex','Location','northeastoutside')
ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,y_gt_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$y_{gt}$','interpreter','latex','Location','northeastoutside')
ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,z_gt_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$z_{gt}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${p_z}$ $[m]$','fontsize',18, 'interpreter','latex')
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,vx_gt_def((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vx_{gt}$','interpreter','latex','Location','northeastoutside')
ylabel('${v_x}$ $[m/s]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,vy_gt_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vy_{gt}$','interpreter','latex','Location','northeastoutside')
ylabel('${v_y}$ $[m/s]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,vz_gt_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$vz_{gt}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${v_z}$ $[m/s]$','fontsize',18, 'interpreter','latex')
% 
% %Plot REFERENCE
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(x_ref((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$x_{ref}$','interpreter','latex','Location','northeastoutside')
% ylabel('${p_x}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(y_ref((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$y_{ref}$','interpreter','latex','Location','northeastoutside')
% ylabel('${p_y}$ $[m]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(z_ref((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$z_{ref}$','interpreter','latex','Location','northeastoutside')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('${p_z}$ $[m]$','fontsize',18, 'interpreter','latex')
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(vx_ref((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$vx_{ref}$','interpreter','latex','Location','northeastoutside')
% ylabel('${v_x}$ $[m/s]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(vy_ref((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$vy_{ref}$','interpreter','latex','Location','northeastoutside')
% ylabel('${v_y}$ $[m/s]$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(vz_ref((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$vz_{ref}$','interpreter','latex','Location','northeastoutside')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% ylabel('${v_z}$ $[m/s]$','fontsize',18, 'interpreter','latex')
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,ax_ref_def((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$ax_{ref}$','interpreter','latex','Location','northeastoutside')
ylabel('${a_x}$ $[m/s^2]$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1, ay_ref_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$ay_{ref}$','interpreter','latex','Location','northeastoutside')
ylabel('${a_y}$ $[m/s^2]$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,az_ref_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$az_{ref}$','interpreter','latex','Location','northeastoutside')
xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
ylabel('${a_z}$ $[m/s^2]$','fontsize',18, 'interpreter','latex')
% 
%Plot metrics
figure('Renderer', 'painters', 'Position', [10 10 900 600])
subplot(3,1,1)
plot(t1,eig_x_def((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
hold on
plot(t1,eig_y_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% plot(t1,eig_z_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$eig_{odom}$','interpreter','latex','Location','northeastoutside')
ylabel('${eig_{x,y,z}}$','fontsize',18, 'interpreter','latex')
subplot(3,1,2)
plot(t1,n_points_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
set(gca,'fontsize',16)
legend('$points$','interpreter','latex','Location','northeastoutside')
ylabel('${points}$','fontsize',18, 'interpreter','latex')
subplot(3,1,3)
plot(t1,trace_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
set(gca,'fontsize',16)
legend('$tr(P)$','interpreter','latex','Location','northeastoutside')
ylabel('${trace}$','fontsize',18, 'interpreter','latex')

% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% subplot(3,1,1)
% plot(t1,eig_velx_def((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% % hold on
% % plot(t1,eig_y_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% % plot(t1,eig_z_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$eig_{odom}$','interpreter','latex','Location','northeastoutside')
% ylabel('${eig_{x,y,z}}$','fontsize',18, 'interpreter','latex')
% subplot(3,1,2)
% plot(t1,eig_vely_def((1:end),1),'Color','[1.00,0.41,0.16]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$points$','interpreter','latex','Location','northeastoutside')
% ylabel('${points}$','fontsize',18, 'interpreter','latex')
% subplot(3,1,3)
% plot(t1,eig_velz_def((1:end),1),'Color','[1.00,0.00,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$tr(P)$','interpreter','latex','Location','northeastoutside')
% ylabel('${trace}$','fontsize',18, 'interpreter','latex')
% figure('Renderer', 'painters', 'Position', [10 10 900 600])
% plot(t1,d_eigx((1:end),1),'Color','[0.07,0.62,1.00]',LineWidth=2)
% set(gca,'fontsize',16)
% legend('$eig_{odom}$','interpreter','latex','Location','northeastoutside')
% ylabel('${\partial eig_{x}$','fontsize',18, 'interpreter','latex')
% xlabel('$t$ $[s]$','fontsize',18,'interpreter','latex')
% off = [1.0,5.0,0.2];
% x_w = X + off(1);
% y_w = Y + off(2);
% R = zeros(length(Qx),1);
% P = zeros(length(Qx),1);
% Yaw = zeros(length(Qx),1);
% for i=1:length(Qx)
%     % R = atan2(2*(Qw*Qx+Qy*Qz),1-2*(Qx^2+Qy^2));
%     % P = -pi/2+atan2(sqrt(1+2*(Qw*Qy-Qx*Qz)),sqrt(1-2*(Qw*Qy-Qx*Qz)));
%     Yaw(i) = atan2(2*(Qw(i)*Qz(i)+Qx(i)*Qy(i)),1-2*(Qy(i)^2+Qz(i)^2));
% end
