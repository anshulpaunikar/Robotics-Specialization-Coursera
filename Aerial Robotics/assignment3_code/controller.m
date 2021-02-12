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
% K_position= [kp_x kd_x; kp_y kd_y; kp_z kd_z];
% K_attitude=[kp_phi kd_phi; kp_theta kd_theta; kp_psi kd_psi];

K_position= [8 7; 8 10; 10 3];
K_attitude=[100 5; 100 5; 100 5];

% Thrust
error_pos=des_state.pos - state.pos;
error_vel=des_state.vel - state.vel;
error1=[error_pos error_vel];

des_acc=des_state.acc+K_position(:,2).*error1(:,2)+K_position(:,1).*error1(:,1);
phi_des=(des_acc(1)*sin(des_state.yaw) - des_acc(2)*cos(des_state.yaw))/params.gravity;
theta_des=(des_acc(1)*cos(des_state.yaw) + des_acc(2)*sin(des_state.yaw))/params.gravity;
psi_des=des_state.yaw;
p_des=0;
q_des=0;
r_des=des_state.yawdot;

F = params.mass*(params.gravity + des_acc(3));

% Moment
error_attitude=[phi_des; theta_des; psi_des] - state.rot;
error_ang_vel= [p_des; q_des; r_des] - state.omega;
error2=[error_attitude error_ang_vel];

M = K_attitude(:,1).*error2(:,1) + K_attitude(:,2).*error2(:,2);

% =================== Your code ends here ===================

end
