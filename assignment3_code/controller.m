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
m=params.mass;
g=params.gravity;

kp_1=500;
kp_2=95;
kp_3=95;

kd_1=0.1;
kd_2=0.1;
kd_3=20;

kp_phi=90;
kp_theta=90;
kp_psi=90;

kd_phi=0;
kd_theta=0;
kd_psi=0;

a1_T=des_state.acc(1);
a2_T=des_state.acc(2);
a3_T=des_state.acc(3);

v1_T=des_state.vel(1);
v2_T=des_state.vel(2);
v3_T=des_state.vel(3);

r1_T=des_state.pos(1);
r2_T=des_state.pos(2);
r3_T=des_state.pos(3);

v1=state.vel(1);
v2=state.vel(2);
v3=state.vel(3);

r1=state.pos(1);
r2=state.pos(2);
r3=state.pos(3);

a1_des= a1_T+kd_1*(v1_T-v1)+ kp_1*(r1_T-r1);
a2_des= a2_T+kd_2*(v2_T-v2)+ kp_2*(r2_T-r2);
a3_des= a3_T+kd_3*(v3_T-v3)+ kp_3*(r3_T-r3);

psi_T=des_state.yaw;

phi_des=(a1_des*sin(psi_T)-a2_des*cos(psi_T))/g;
theta_des=(a1_des*cos(psi_T)+a2_des*sin(psi_T))/g;
psi_des=psi_T;

phi=state.rot(1);
theta=state.rot(2);
psi=state.rot(3);

p=state.omega(1);
q=state.omega(2);
r=state.omega(3);

r_des=des_state.yawdot;

x=kp_phi*(phi_des-phi)+ kd_phi*(0-p);
y=kp_theta*(theta_des-theta)+ kd_theta*(0-q);
z=kp_psi*(psi_des-psi)+ kd_psi*(r_des-r);


u1= m*g+ m*a3_des;
u2=[x;y;z];
% Thrust
F =u1;

% Moment
%M = zeros(3,1);
M=u2;

% =================== Your code ends here ===================

end
