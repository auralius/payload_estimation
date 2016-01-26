%% Cleaning up, warning!!
clear all
clc

%% Parameters to estimate. Later, compare the estimation results with these:
% Ixx = ;
% Ixy = ;
% Ixz = ;
% Ixx = ;
% Iyy = ;
% Izz = ;
% Inertias = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
Inertias = eye(3); % kg*m^2
m = 0.765; % kg
CoM = [0.058 -0.007 0]; % m

%% Do the simulation
sim_time = 50;
ts = 0.0050;

% This is using MATLAB R2015b
sim('model_6_dof.slx');