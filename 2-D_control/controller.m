function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
%define params
g = params.gravity;
m = params.mass;
Ixx = params.Ixx;
l = params.arm_length;
minF = params.minF;
maxF = params.maxF;

%current state 
y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);
phi = state.rot;
phi_dot = state.omega;

%desired state
y_des = des_state.pos(1);
z_des = des_state.pos(2);
y_dot_des = des_state.vel(1);
z_dot_des = des_state.vel(2);
y_ddot_des = des_state.acc(1);
z_ddot_des = des_state.acc(2);

%gains
Kp_y = 20; Kv_y = 5;
Kp_z = 300; Kv_z = 30;
Kp_phi = 2000; Kv_phi = 50;

%code
phi_des = (-1/g)*(y_ddot_des + Kp_y*(y_des - y) + Kv_y*(y_dot_des - y_dot));
phi_dot_des = 0;
phi_ddot_des = 0;
u1 = m*(g + z_ddot_des + Kp_z*(z_des - z) + Kv_z*(z_dot_des - z_dot));
u2 = Ixx*(phi_ddot_des + Kp_phi*(phi_des - phi) + Kv_phi*(phi_dot_des - phi_dot))
end

