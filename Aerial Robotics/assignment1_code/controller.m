function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters


% FILL IN YOUR CODE HERE
z2_des=0;
kp=120;kv=15;
e=s_des-s;
u = min(max(params.u_min,params.mass*(z2_des+kp*e(1)+kv*e(2)+params.gravity)),params.u_max);
end

