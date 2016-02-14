function [ F, M ] = controller(~, state, des_state, params)
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

F = 0;
M = 0;

% FILL IN YOUR CODE HERE
Kvz=10;
Kpz=800;
Kvphi=25;
Kpphi=1600;
Kvy=10;
Kpy=40;

F = params.mass*(params.gravity ...
    + des_state.acc(2) ...
    + Kvz*(des_state.vel(2)-state.vel(2)) ...
    + Kpz*(des_state.pos(2)-state.pos(2)));

phi_c = -1/params.gravity*(des_state.acc(1) ...
    + Kvy*(des_state.vel(1)-state.vel(1)) ...
    + Kpy*(des_state.pos(1)-state.pos(1)));

M = params.Ixx*(0 ...
    + Kvphi*(0-state.omega) ...
    + Kpphi*(phi_c-state.rot));

% test for limits and force them
if F<params.minF
    F=params.minF;
end
if F>params.maxF
    F=params.maxF;
end

end

