clear all
close all
clc

%% Package paths

cur = pwd;
addpath( genpath( [cur, '/gen/' ] ));

%% testing with generated data
% Note the values used are totally made up and shouldn't be used as
% starting points for your actual analysis

% alpha = 0.1;
% gamma = 0.5;
% t = 0:0.01:2;
% u = 150*ones(size(t));
% tau0 = 0;
% dtau0 = 0;
% tau = find_tau(u, t, alpha, gamma, tau0, dtau0);

% K = 3;
% D = 0.4;
% q0 = 0;
% dq0 = 0;
% q = find_q(tau, t, K, D, q0, dq0);


% x = K, D, alpha, gamma

% cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);


% [X, resnorm] = lsqnonlin(cost, [2.6, 0.5, 0.31, 0.6]);
% [X, resnorm] = lsqnonlin(cost, [K, D, alpha, gamma])


%% 

% clear all
% close all
% clc

%% Load CSV
data = importdata('/Users/ebronstein/Documents/spring2019/ee106b/labs/lab4/lab4_pkg/data/processed_system_id/150.0_0.csv');

%% Assume initial steady state
q0 = 0;
dq0 = 0;
tau0 = 0;
dtau0 = 0;

%% Generate q(t)
num_samples = length(data.time);
% idx = randsample(1:size(data.time), num_samples);
t = data.time(1:num_samples);
u = data.right_pwm(1:num_samples);
q = bend_angle_from_flex(data.right_flex(1:num_samples)); % estimate bend angle from flex (i.e. resistance)

%% Define cost
cost = @(x) cost_function(x(1), x(2), x(3), x(4), t, u, q, q0, dq0, tau0, dtau0);
cost_arrayfun = @(x1, x2, x3, x4) cost_function(x1, x2, x3, x4, t, u, q, q0, dq0, tau0, dtau0);
% cost(0.01, 0.01, 0.01, 0.01)
% cost_function(1., 1., 1., 1., t, u, q, q0, dq0, tau0, dtau0)

%% Gridding
% K_grid = linspace(1e-4, 0.1, 10);
% D_grid = linspace(1e-4, 0.1, 10);
% alpha_grid = linspace(1e-8, 1e-4, 10);
% gamma_grid = linspace(1e-6, 0.1, 10);
% [K_mesh, D_mesh, alpha_mesh, gamma_mesh] = ndgrid(K_grid, D_grid, alpha_grid, gamma_grid);
% costvals = arrayfun(cost_arrayfun, K_mesh, D_mesh, alpha_mesh, gamma_mesh);

%% do your regression
K0 = 0.0976; % 5e-3;
D0 = 0.01; % 1e-3;
alpha0 = 0.0024; % 5e-5;
gamma0 = 0.01; % 1e-3;

x0 = [K0, D0, alpha0, gamma0];
initial_cost = cost(x0);
lb = zeros(size(x0));
ub = 10. * ones(size(x0));
fprintf("Beginning optimization...");
[X, resnorm, residual, exitflag, output, lambda, jac] = lsqnonlin(cost, x0, lb, ub)



