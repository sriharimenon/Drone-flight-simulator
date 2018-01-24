function [F, M, trpy, drpy, d] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
% phi_des = 0;
% theta_des = 0;
% psi_des = 0;

%persistent gd;

d=[0 0 0];
Inertia=[1.43e-05,0,0;0,1.43e-05,0;0,0,2.89e-05];
g=9.81;

%gains
Kp=[5,5,200,2400,2400,200];           %1->r_acc_1; 2->r_acc_2; 3->r_acc_3; 
Kd=[4,4,78,50,50,50];                %4->rot_phi; 5->rot_theta; 6->rot_psi;

%note: here, des means T( in literature) and commanded means des( in literature)
r_acc_commanded(1) = qd{qn}.acc_des(1)+Kd(1)*(qd{qn}.vel_des(1)-qd{qn}.vel(1))+Kp(1)*(qd{qn}.pos_des(1)-qd{qn}.pos(1));
r_acc_commanded(2) = qd{qn}.acc_des(2)+Kd(2)*(qd{qn}.vel_des(2)-qd{qn}.vel(2))+Kp(2)*(qd{qn}.pos_des(2)-qd{qn}.pos(2));
r_acc_commanded(3) = qd{qn}.acc_des(3)+Kd(3)*(qd{qn}.vel_des(3)-qd{qn}.vel(3))+Kp(3)*(qd{qn}.pos_des(3)-qd{qn}.pos(3));

% Thurst
u1   = params.mass*(r_acc_commanded(3)+g);
F    = u1;

%desired attitude

phi_des = (1/g)*(r_acc_commanded(1)*sin(qd{qn}.yaw_des)-r_acc_commanded(2)*cos(qd{qn}.yaw_des));
theta_des = (1/g)*(r_acc_commanded(1)*cos(qd{qn}.yaw_des)+r_acc_commanded(2)*sin(qd{qn}.yaw_des));
psi_des = qd{qn}.yaw_des;

p_des=0;
q_des=0;
r_des=qd{qn}.yawdot_des;

%gd = [gd; t, phi_des, qd{qn}.euler(1)];  % for graphing

u2=Inertia*[Kp(4)*(phi_des-qd{qn}.euler(1))+Kd(4)*(p_des-qd{qn}.omega(1));
            Kp(5)*(theta_des-qd{qn}.euler(2))+Kd(5)*(q_des-qd{qn}.omega(2));
            Kp(6)*(psi_des-qd{qn}.euler(3))+Kd(6)*(r_des-qd{qn}.omega(3));];

% Moment
M    = u2;
% =================== Your code ends here ===================

% if t > 4.4
%      figure(4)
%      % Desired in red
%      plot(gd(:, 1), gd(:, 2), 'r')
%      hold on
%      % Actual in blue
%      plot(gd(:,1), gd(:, 3), 'b');
%      hold off
%      legend('Desired angle', 'Actual angle')
% end

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
