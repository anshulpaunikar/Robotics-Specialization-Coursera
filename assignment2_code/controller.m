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


% FILL IN YOUR CODE HERE
% kp_y=0.2;kp_z=3;kp_phi=10;
% kd_y=0.8;kd_z=5;kd_phi=0.8;
kp_y=20;kp_z=80;kp_phi=1000;
kd_y=15;kd_z=20;kd_phi=100;

error_vel=des_state.vel-state.vel;
error_pos=des_state.pos-state.pos;
phi_c_dot=0; phi_c2_dot=0;

phi_c=-1/params.gravity*(des_state.acc(1)+kd_y*error_vel(1)+kp_y*error_pos(1));
u1=params.mass*(params.gravity+des_state.acc(2)+kd_z*error_vel(2)+kp_z*error_pos(2));
u2=params.Ixx*(phi_c2_dot+kd_phi*(phi_c_dot-state.omega)+kp_phi*(phi_c-state.rot));
end

