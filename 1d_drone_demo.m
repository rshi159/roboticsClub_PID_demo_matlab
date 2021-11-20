close all; clc; clear all;
%% Variables
% position
x = [[0,5]];
% velocity
x_dot = [0,0];
% acceleration
x_ddot = [0,0];
% contol input
u = [0,0];

% gravitational acceleration
G = [0,-9.81];

start = [0,5];
goal = [0,10];

sim_duration = 10;
dt = 0.1;
%% Dynamics Update
% x = x + x_dot*dt;
% x_dot = x_dot + x_ddot*dt;
% x_ddot = u;
for i = 2 : sim_duration/dt
    x(i) = x(i-1) + x_dot*dt;
end
%% plot figure
f1 = figure(1);
subplot(3,1,1)
plot()