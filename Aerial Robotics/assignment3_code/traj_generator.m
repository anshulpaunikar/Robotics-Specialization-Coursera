function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here
persistent waypoints0 traj_time d0 alpha poly_coeff
if nargin > 2
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;

        N = size(waypoints, 2)-1; % Num of polynomial pieces

        % Coeffs matrix of the polynomial
        poly_coeff = zeros(7, 8); % Coefficients of the polynomial

        poly_coeff(1,:) = ones(1,8); % Coeffs of the poly

        p = poly2sym(poly_coeff(1,:));

        poly_diff1 = diff(p); % Coeffs of the 1st diff
        poly_coeff(2,2:8) = coeffs(poly_diff1);
        poly_diff2 = diff(poly_diff1);
        poly_coeff(3,3:8) = coeffs(poly_diff2);
        poly_diff3 = diff(poly_diff2);
        poly_coeff(4,4:8) = coeffs(poly_diff3);
        poly_diff4 = diff(poly_diff3);
        poly_coeff(5,5:8) = coeffs(poly_diff4);
        poly_diff5 = diff(poly_diff4);
        poly_coeff(6,6:8) = coeffs(poly_diff5);
        poly_diff6 = diff(poly_diff5);
        poly_coeff(7,7:8) = coeffs(poly_diff6);

        start_pt = diag(poly_coeff);
        start_pt = diag(start_pt);
        start_pt = [start_pt,zeros(7,1)];

        A = zeros(8*N);
        b = zeros(8*N, 3);
        for i = 1:N
            A((i-1)*8+1, (1:8)+(i-1)*8) = start_pt(1,:);
            b((i-1)*8+1, :) = waypoints(:,i)';

            A((i-1)*8+2, (1:8)+(i-1)*8) = poly_coeff(1,:);
            b((i-1)*8+2, :) = waypoints(:,i+1)';

            if i < N
                A((i-1)*8+3, (1:16)+(i-1)*8) = [poly_coeff(2,:), -start_pt(2,:)];
                A((i-1)*8+4, (1:16)+(i-1)*8) = [poly_coeff(3,:), -start_pt(3,:)];
                A((i-1)*8+5, (1:16)+(i-1)*8) = [poly_coeff(4,:), -start_pt(4,:)];
                A((i-1)*8+6, (1:16)+(i-1)*8) = [poly_coeff(5,:), -start_pt(5,:)];
                A((i-1)*8+7, (1:16)+(i-1)*8) = [poly_coeff(6,:), -start_pt(6,:)];
                A((i-1)*8+8, (1:16)+(i-1)*8) = [poly_coeff(7,:), -start_pt(7,:)];
            end

        end

        A(8*N-5, 1:8) = start_pt(2,:);
        A(8*N-4, 1:8) = start_pt(3,:);
        A(8*N-3, 1:8) = start_pt(4,:);
        A(8*N-2, (1:8)+8*(N-1)) = poly_coeff(2,:);
        A(8*N-1, (1:8)+8*(N-1)) = poly_coeff(3,:);
        A(8*N  , (1:8)+8*(N-1)) = poly_coeff(4,:);

        alpha(:,:,1) = reshape(A\b(:,1), 8, N);
        alpha(:,:,2) = reshape(A\b(:,2), 8, N);
        alpha(:,:,3) = reshape(A\b(:,3), 8, N);
    else
        if(t > traj_time(end))
            t = traj_time(end);
        end
        t_index = find(traj_time >= t,1);

        if(t_index > 1)
            t = t - traj_time(t_index-1);
        end
        if(t == 0)
            desired_state.pos = waypoints0(:,1);
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
        else
            scale = t/d0(t_index-1);

            f_p = squeeze(alpha(:,t_index-1,:))'.*repmat(poly_coeff(1,:),3,1);
            f_p = flip(f_p,2);
            desired_state.pos = [polyval(f_p(1,:),scale);
                                 polyval(f_p(2,:),scale);
                                 polyval(f_p(3,:),scale)];

            f_v = squeeze(alpha(:,t_index-1,:))'.*repmat(poly_coeff(2,:),3,1);
            f_v = flip(f_v(:,2:8),2);
            desired_state.vel = [polyval(f_v(1,:),scale);
                                 polyval(f_v(2,:),scale);
                                 polyval(f_v(3,:),scale)]/d0(t_index-1); 

            f_a = squeeze(alpha(:,t_index-1,:))'.*repmat(poly_coeff(3,:),3,1); 
            f_a = flip(f_a(:,3:8),2);
            desired_state.acc = [polyval(f_a(1,:),scale);
                                 polyval(f_a(2,:),scale);
                                 polyval(f_a(3,:),scale)]/(d0(t_index-1))^2;

        end

        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
end

