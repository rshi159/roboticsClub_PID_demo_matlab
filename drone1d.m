close all; clc; clear;
%% 
% This file simulates a drone flying in 1 dimension. We will use this
% example to learn more about PID control.

% Feel free to play around with the kp, ki, kd terms to see what happens.
% You can turn on gravity be setting G = [0, -9.8]
% Try extending this to a 2 dimensional system.

%% Variables and Setup
% This variable keeps track of position over time.
x(1,:) = [0,8];
initial_position = x(1,:);
% This variable keeps track of velocity over time.
x_dot(1,:) = [0,0];
% This variable keeps track of acceleration over time.
x_ddot(1,:) = [0,0];
% U is our control input. For this system, think of it as adjusting the
% thrust of the drone. We cannot instantaneously change the position or the
% velocity this way, but it will affect the acceleration of the drone.
u(1,:) = [0,0];
% We keep track of the time to be able to animate the motion of the drone.
t(1,:) = [0];
% This variable keeps track of the past error values. It is used for
% getitng the integral and derivative terms.
error = [];

% control limits in [x,y]. For our 2D example, we can only move in the y
% direction. We give some acceleration limits that may depend on a bunch of
% parameters like motor type and direction of rotation, propeller design,
% profile of the drone body, ...
u_max = [0,25];
u_min = [0,-25];

% gravitational acceleration constant. it only acts in the -y direction.
G = [0,0];%[0,-9.81];

% we define the [x,y] coordinates of the start and the goal. Since we only
% specify the y value, this is like keeping a drone hovering at a certain
% height.
start = [0,10];
goal = [0,15];

% these values determine how long the simulation runs for. sim_duration is
% the amount of simulation time for the drone flight. dt is the timestep.
% So the total number of steps that we will simulate is sim_duration/dt.
sim_duration = 15;
dt = 0.05;
%% function declaration
% we will implement our function for u in a function called controller. It
% takes inputs of:
% x which is the current position of the drone.
% x_dot which is the velocity of the drone
% x_ddot which is the acceleration of the drone.
% goal which is the position of the goal state.
% error is an array that holds the history of errors in previous time
    % slices.
u = controller(x, goal,dt, error);
%% Dynamics Update
% These equations dictate how the dynamics of the drone change. We really
% only care about maintaing a certain position and how our changes to the
% acceleration affect that position.

% the position of the drone depends on the velocity of the past time step.
% x(t+1) = x(t) + x_dot(t)*dt;

% the velocity depends on the acceleration in the past time step.
% x_dot(t+1) = x_dot(t) + x_ddot(t)*dt;

% and we can control the acceleration. The acceleration is also affected by
% a magical thing called "gravity".
% x_ddot = u + G;

%% This function displays the agent.

% drone_color is cosmetic. it makes the drone a certain color, defaulted to
% 'g' for green. See the matlab documentation on plots and rectangles to
% see how this variable is used.
drone_color = 'g';
% This for loop over all timesteps of the simulation.
% sim_t is the loop index that represents the current time of the
% simulation.
% linspace takes inputs of start time, end time, and number of time steps.
% This gives us an array of all the times of the simulation. The for loop
% just iterates over every value in the array.
% run this code fragment to see the outputs:
% for t = linspace(1,10,10)
%     disp(t)
% end
for sim_t = linspace(dt, sim_duration, sim_duration/dt)
    % i is used for indexing. We start at index 2 since index 1 is used for
    % the initial conditions of the system, ie the initial position,
    % initial velocity, ... Remember, matlab is is 1 indexed!
    i = int32(sim_t/dt + 1);
    % this display acts like a print. It will display the current
        % simulation time.
    if mod(i,10)==1
        disp("simulation time: " + sim_t);
    end
    % Update the time for plotting.
    t(i,:) = t(i-1,:) + dt;
    % update dynamics, using the equations we described at the top of this
    % section. Think discrete time integration.
    x(i,:) = x(i-1,:) + x_dot(i-1,:)*dt;
    x_dot(i,:) = x_dot(i-1,:) + x_ddot(i-1,:)*dt;

    % use the controller.
    [u(i,:),error] = controller(x(i-1,:), goal,dt, error);
% check control limits. We cannot exceed the limits of our systen.
    if u(i,2) >= u_max(2)
        x_ddot(i,:) = u_max;
    elseif u(i,2) <= u_min(2)
        x_ddot(i,:) = u_min;
    else
        x_ddot(i,:) = u(i,:);
    end
    % factor in acceleration due to gravity.
    x_ddot(i,:) = x_ddot(i,:)+G;
    
    % collision check. We don't want to crash into the ground. We can add
    % more checks if we want to add walls or ceilings.
    if x(i,2) <= 0
        disp("Collision");
        x(i,2) = 0;
        drone_color = 'r';
    end
     
% plot the drone
    % create the figure window.
    f1 = figure(1);
    % plot the drone.
    p = plot(x(i,1),x(i,2),'--gs',...
    'LineWidth',5,...
    'MarkerSize',10,...
    'MarkerEdgeColor',drone_color,...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    yline(goal(2));
    yline(0);
    axis([-10 10 0 25]);
    grid on;
    % end the simulation if we crash into the ground.
    if x(i,2) <= 0
        break;
    end
end
% plot the values for the entire simulation.
f2 = figure(2);
% subplots allow for many plots. We separate a figure window into 4 rows
% and 1 column. The last input determines the index of the plot. So the
% next line says that in the plot window divided into 4 rows and 1 column,
% plot the stuff in the first plot.
subplot(5,1,1)
hold on;
plot(t, x(:,1),'r', "linewidth", 3);
plot(t, x(:,2),'b', "linewidth", 3);
xlabel("time (s)");
ylabel("position (m)");
legend(["x","y"]);
set(gca,'fontsize',12);
grid on;
hold off;

%velocity plot
subplot(5,1,2)
hold on;
plot(t, x_dot(:,1),'r', "linewidth", 3);
plot(t, x_dot(:,2),'b', "linewidth", 3);
xlabel("time (s)");
ylabel("velocity (m/s)");
legend(["x","y"]);
set(gca,'fontsize',12);
grid on;
hold off;
    
%acceleration plot
subplot(5,1,3)
hold on;
plot(t, x_ddot(:,1),'r', "linewidth", 3);
plot(t, x_ddot(:,2),'b', "linewidth", 3);
xlabel("time (s)");
ylabel("acceleration (m/s^2)");
legend(["x","y"]);
set(gca,'fontsize',12);
grid on;
hold off;

%control input plot
subplot(5,1,4)
hold on;
plot(t, u(:,1),'r', "linewidth", 3);
plot(t, u(:,2),'b', "linewidth", 3);
xlabel("time (s)");
ylabel("control (m/s^2)");
legend(["x","y"]);
set(gca,'fontsize',12);
grid on;
hold off;

% errror plot
subplot(5,1,5)
hold on;
plot(t(1:end-1,:), error(:,1),'r', "linewidth", 3);
plot(t(1:end-1,:), error(:,2),'b', "linewidth", 3);
xlabel("time (s)");
ylabel("error (m)");
legend(["x","y"]);
set(gca,'fontsize',12);
grid on;
hold off;

% the controller takes inputs of current position and past error
% and calculates the best control input to send.
function [u,error] = controller(x, goal,dt, error)
%     %u = [0,0];
%     error = error;
%     %Example of bang bang control in 1D
%     if x(2) <= goal(2)
%         u = [0,15];
%     elseif x(2) > goal(2)
%         u = [0,0];
%     else
%         u = [0,0];
%     end

    %PID 
    % the error of the current timeslice 
    e = x-goal;
    % calculate the derivative term. We have to check if therer is a
    % previous error term, or there is an array out of bounds exception.
    if numel(error)==0
        d = 0;
    else
        d = (e-error(end,:))/dt;
    end
    % append the current error to the end of the error array.
    error = [error; e];
    % PID constants. Play around with these.
    kp = 5;
    ki = 2;
    kd = 1.25;
    % we plug everything into the equation. Note that we take the negative
    % sign. If the robot is abo
    u = -(kp*e + ki/numel(error)*sum(error) + kd * d);
end