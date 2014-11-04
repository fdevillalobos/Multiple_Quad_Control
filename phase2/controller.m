function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega

% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des

% =================== Your code goes here ===================

% Rotation Matrix
roll  = qd{qn}.euler(1);
pitch = qd{qn}.euler(2);
yaw   = qd{qn}.euler(3);

% Rotation Matrix - I only used R(3,3).
R = [cos(yaw)*cos(pitch) - sin(roll)*sin(yaw)*sin(pitch),-cos(roll)*sin(yaw),cos(yaw)*sin(pitch) + cos(pitch)*sin(roll)*sin(yaw);
     cos(pitch)*sin(yaw) + cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch) - cos(yaw)*cos(pitch)*sin(roll);
     -cos(roll)*sin(pitch)                              , sin(roll)         , cos(roll) * cos(pitch)];
 
%R_33 = cos(roll) * cos(pitch);

% Error Control     Optimal Values: [16 12] [16 12] [16 10];    
% Real Values: [5 4] [5 4] [10 4] Z coord takes up to 17 in real quad.
kp = 25;     kd = 20;
r1_2dot = kd * (qd{qn}.vel_des(1) - qd{qn}.vel(1)) + kp * (qd{qn}.pos_des(1) - qd{qn}.pos(1))  + qd{qn}.acc_des(1);
kp = 25;     kd = 20;
r2_2dot = kd * (qd{qn}.vel_des(2) - qd{qn}.vel(2)) + kp * (qd{qn}.pos_des(2) - qd{qn}.pos(2))  + qd{qn}.acc_des(2);
kp = 30;     kd = 25;
r3_2dot = kd * (qd{qn}.vel_des(3) - qd{qn}.vel(3)) + kp * (qd{qn}.pos_des(3) - qd{qn}.pos(3))  + qd{qn}.acc_des(3);

% Thurst
F    = params.mass * (params.grav + r3_2dot) / R(3,3);

% Desired roll, pitch and yaw
phi_des   = 1/params.grav* (r1_2dot* sin(qd{qn}.yaw_des) - r2_2dot* cos(qd{qn}.yaw_des));
theta_des = 1/params.grav* (r1_2dot* cos(qd{qn}.yaw_des) + r2_2dot* sin(qd{qn}.yaw_des));
psi_des   = qd{qn}.yaw_des;

% Moment
kp_phi   =  0.1;     kd_phi   = kp_phi/2;
kp_theta =  0.1;     kd_theta = kp_theta/2;
kp_psi   =  0.1;     kd_psi   = kp_psi/2;
p_des    =  0;     q_des    = 0;     r_des = qd{qn}.yawdot_des;

M    = [kp_phi  *(phi_des  -qd{qn}.euler(1)) + kd_phi   * (p_des - qd{qn}.omega(1));
        kp_theta*(theta_des-qd{qn}.euler(2)) + kd_theta * (q_des - qd{qn}.omega(2));
        kp_psi  *(psi_des  -qd{qn}.euler(3)) + kd_psi   * (r_des - qd{qn}.omega(3))];

%M = zeros(3,1);

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
