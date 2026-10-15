%% ======================================================================
%  Paper Fig 4 and Fig 5 Reproduction: Algorithm Comparison under Different Step Sizes (No Disturbance)
%  Fig 4: Regulation error ek = |x| + ν²
%  Fig 5: Control amplitude |u|
%  Step sizes compared: h = 0.001 and h = 0.1
%  Algorithms compared: ISTA and ESTA
%  Note: Both under no disturbance case (Δ(t) = 0)
% ======================================================================

clear; clc; close all;
format long;

%% === Controller Parameter Settings ===
lambda_1 = 10;      % Control gain λ₁
lambda_2 = 6;       % Control gain λ₂
T_end = 5;         % Total simulation time [seconds]

%% === Initial Conditions ===
x_0 = 10;           % Initial state
nu_0 = 10;          % Controller internal integral variable initial value
phi_0 = 0;          % Disturbance integral initial value

%% === Step Size Settings ===
h_small = 0.001;   % Small step size
h_large = 0.1;     % Large step size

%% ========== No Disturbance Simulation ==========
fprintf('======== No Disturbance Simulation ========\n');
disturbance_off = @(t) 0;  % No disturbance

% --- Small step size h=0.001 ---
fprintf('Running h=0.001...\n');
[t1, x_ista1, u_ista1, nu_ista1, ek_ista1, x_esta1, u_esta1, nu_esta1, ek_esta1] = ...
    run_simulation(h_small, T_end, x_0, nu_0, phi_0, lambda_1, lambda_2, disturbance_off);

% --- Large step size h=0.1 ---
fprintf('Running h=0.1...\n');
[t2, x_ista2, u_ista2, nu_ista2, ek_ista2, x_esta2, u_esta2, nu_esta2, ek_esta2] = ...
    run_simulation(h_large, T_end, x_0, nu_0, phi_0, lambda_1, lambda_2, disturbance_off);

fprintf('\nSimulation completed!\n');

%% ========== Fig 4 Plot: Regulation error ek =  |x| + ν² ==========
figure('Name','Fig 4: Regulation Error ek (No Disturbance)','Color','w','Position',[50 50 1400 500]);

% Small step size h=0.001
subplot(1,2,1);
semilogy(t1, ek_ista1, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t1, ek_esta1, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Regulation Error e_k (log)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig 4: Regulation Error - h=%.4f', h_small));

% Large step size h=0.1
subplot(1,2,2);
semilogy(t2, ek_ista2, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t2, ek_esta2, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Regulation Error e_k (log)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig 4: Regulation Error - h=%.2f', h_large));

%% ========== Fig 5 Plot: Control amplitude |u| ==========
figure('Name','Fig 5: Control Amplitude (No Disturbance)','Color','w','Position',[100 100 1400 500]);

% Small step size h=0.001
subplot(1,2,1);
semilogy(t1, abs(u_ista1) + 1e-10, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t1, abs(u_esta1) + 1e-10, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Control Amplitude |u| (log)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig 5: Control Amplitude - h=%.4f', h_small));

% Large step size h=0.1
subplot(1,2,2);
semilogy(t2, abs(u_ista2) + 1e-10, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t2, abs(u_esta2) + 1e-10, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Control Amplitude |u| (log)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig 5: Control Amplitude - h=%.2f', h_large));

%% ========== Performance Metrics Comparison ==========
fprintf('\n========== Performance Comparison Results (No Disturbance) ==========\n');
print_metrics('h=0.001', x_ista1, u_ista1, x_esta1, u_esta1, h_small, T_end);
print_metrics('h=0.1  ', x_ista2, u_ista2, x_esta2, u_esta2, h_large, T_end);

%% ========== Helper Function: Main Simulation Loop ==========
function [t_log, x_ista, u_ista, nu_ista, ek_ista, x_esta, u_esta, nu_esta, ek_esta] = ...
    run_simulation(h, T_end, x0, nu0, phi0, lambda1, lambda2, disturbance)

    N = floor(T_end/h);
    t_log = (0:N-1)*h;

    % ISTA data
    x_ista = zeros(1,N);
    u_ista = zeros(1,N);
    nu_ista = zeros(1,N);
    ek_ista = zeros(1,N);

    % ESTA data
    x_esta = zeros(1,N);
    u_esta = zeros(1,N);
    nu_esta = zeros(1,N);
    ek_esta = zeros(1,N);

    %% ISTA simulat
    x = x0; nu = nu0; phi = phi0;
    for k = 1:N
        t = (k-1)*h;
        % Delta = disturbance(t);
        Delta = 0;

        [u, nu_next, sqrt_tilde_x] = ISTA(x, nu, lambda1, lambda2, h);
        x_next = x + h*(u + phi);
        phi_next = phi + h*Delta;

        x_ista(k) = x_next;
        u_ista(k) = u;
        nu_ista(k) = nu;
        ek_ista(k) = abs(x_next) + nu^2;  % Regulation error: ek = |x| + ν²

        x = x_next;
        nu = nu_next;
        phi = phi_next;
    end

    %% ESTA simulat
    x = x0; nu = nu0; phi = phi0;
    for k = 1:N
        t = (k-1)*h;
        % Delta = disturbance(t);
        Delta = 0;

        [u, nu_next] = ESTA(x, nu, lambda1, lambda2, h);
        x_next = x + h*(u + phi);
        phi_next = phi + h*Delta;

        x_esta(k) = x_next;
        u_esta(k) = u;
        nu_esta(k) = nu;
        ek_esta(k) = abs(x_next) + nu^2;  % Regulation error: ek = |x| + ν²

        x = x_next;
        nu = nu_next;
        phi = phi_next;
    end
end

%% ========== Helper Function: Performance Metrics Printing ==========
function print_metrics(label, x_ista, u_ista, x_esta, u_esta, h, T_end)
    fprintf('\nstep %s:\n', label);
    fprintf('Algorithm\t\tTerminal Error\t\tControl Energy\t\tPeak Control\n');
    fprintf('------------------------------------------------------------\n');

    % ISTA index
    final_err_ista = abs(x_ista(end));
    energy_ista = sum(u_ista.^2) * h;
    peak_ista = max(abs(u_ista));

    % ESTA index
    final_err_esta = abs(x_esta(end));
    energy_esta = sum(u_esta.^2) * h;
    peak_esta = max(abs(u_esta));

    fprintf('ISTA\t\t%.4e\t\t%.4f\t\t%.4f\n', final_err_ista, energy_ista, peak_ista);
    fprintf('ESTA\t\t%.4e\t\t%.4f\t\t%.4f\n', final_err_esta, energy_esta, peak_esta);
end
