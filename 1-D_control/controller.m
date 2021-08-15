function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;


% FILL IN YOUR CODE HERE
m = params.mass;
g = params.gravity;
u_min = params.u_min;
u_max = params.u_max;
e_z = s_des(1) - s(1);
de_z = s_des(2) - s(2);
Kp = 300;
Kv = 30;

d2z_des = 0;
u = m * (d2z_des + Kp*e_z + Kv*de_z + g);
end

