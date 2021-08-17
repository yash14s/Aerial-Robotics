function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%robot params
m = params.mass;
I = params.I;
invI = params.invI;
g = params.gravity;
l = params.arm_length;
minF = params.minF; maxF = params.maxF;

%state
x = state.pos(1); y = state.pos(2); z = state.pos(3);
x_dot = state.vel(1); y_dot = state.vel(2); z_dot = state.vel(3);
phi = state.rot(1); theta = state.rot(2); psi = state.rot(3);
p = state.omega(1); q = state.omega(2); r = state.omega(3);

%desired state
x_des = des_state.pos(1); y_des = des_state.pos(2); z_des = des_state.pos(3);
x_dot_des = des_state.vel(1); y_dot_des = des_state.vel(2); z_dot_des = des_state.vel(3);
x_ddot_des = des_state.acc(1); y_ddot_des = des_state.acc(2); z_ddot_des = des_state.acc(3);
psi_des = des_state.yaw; r_des = des_state.yawdot;

%gains
Kp_x = 100; Kd_x = 20;
Kp_y = 100; Kd_y = 20;
Kp_z = 300; Kd_z = 30;
Kp_phi = 2000; Kd_phi = 50;
Kp_theta = 2000; Kd_theta = 50;
Kp_psi = 2000; Kd_psi = 50;

%commanded acc
x_ddot = x_ddot_des + Kp_x*(x_des-x) + Kd_x*(x_dot_des-x_dot);
y_ddot = y_ddot_des + Kp_y*(y_des-y) + Kd_y*(y_dot_des-y_dot);
z_ddot = z_ddot_des + Kp_z*(z_des-z) + Kd_z*(z_dot_des-z_dot);

%commanded pitch, roll
phi_des = (1/g)*(x_ddot*sin(psi_des) - y_ddot*cos(psi_des));
theta_des = (1/g)*(x_ddot*cos(psi_des) + y_ddot*sin(psi_des));
p_des = 0; q_des = 0;

% Thrust
F = m*(g + z_ddot);

% Moment
M = [Kp_phi*(phi_des-phi) + Kd_phi*(p_des - p);
     Kp_theta*(theta_des-theta) + Kd_theta*(q_des - q);
     Kp_psi*(psi_des-psi) + Kd_psi*(r_des - r);];
% =================== Your code ends here ===================

end
