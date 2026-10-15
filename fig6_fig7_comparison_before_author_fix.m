%% ======================================================================
% Reproduction of Figs. 6 and 7 from the paper: Algorithm comparison under disturbance
% Fig. 6: Regulation error e_k = |x| + ν^2
% Fig. 7: Control amplitude |u|
% Disturbance: Δ(t) = 2(1 + sin(2t^2))
% Step size: h = 0.001
% Initial condition: φ(0) = 0
% Algorithms compared: ISTA and ESTA
% Conclusions from the paper:
%   - ISTA (implicit method) has a smaller control amplitude, less chattering,
%     and higher accuracy.
%   - ESTA (explicit method) exhibits larger oscillations in the presence of disturbances.
% ======================================================================

clear; clc; close all;

%% === Controller Parameter Settings ===
lambda_1 = 10;      % Control gain λ₁
lambda_2 = 6;       % Control gain λ₂
T_end = 5;          % Total simulation time [s]

%% === Initial Conditions ===
x_0 = 10;           % Initial state
nu_0 = 10;          % Initial value of the controller's internal integral variable
phi_0 = 0;          % Initial value of the disturbance integral, φ(0) = 0

%% === Step Size Settings ===
h = 0.001;          % Step size

%% ========== Simulation with Disturbance ==========
fprintf('======== Simulation with Disturbance ========\n');
disturbance = @(t) 2*(1 + sin(2*t^2));  % Disturbance: Δ(t) = 2(1 + sin(2t²))

%% disturbance = @(t) 0;  % No disturbance

% --- Step size h = 0.001 ---
fprintf('Running h = 0.001...\n');
[t, x_ista, u_ista, nu_ista, ek_ista, x_esta, u_esta, nu_esta, ek_esta] = ...
    run_simulation(h, T_end, x_0, nu_0, phi_0, lambda_1, lambda_2, disturbance);
fprintf('\nSimulation completed!\n');

%% ========== Fig. 6: Regulation Error \(e_k = |x| + \nu^2\) ==========
figure('Name', 'Fig. 6: Regulation Error e_k (with disturbance)', ...
       'Color', 'w', 'Position', [50 50 800 600]);
semilogy(t, ek_ista, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t, ek_esta, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Regulation Error e_k (log scale)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig. 6: Regulation Error - h = %.4f (with disturbance)', h));

%% ========== Fig. 7: Control Amplitude \(|u|\) ==========
figure('Name', 'Fig. 7: Control Amplitude (with disturbance)', ...
       'Color', 'w', 'Position', [100 100 800 600]);
semilogy(t, abs(u_ista) + 1e-10, 'b-', 'LineWidth', 1.5); hold on;
semilogy(t, abs(u_esta) + 1e-10, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]'); ylabel('Control Amplitude |u| (log scale)');
legend('ISTA', 'ESTA', 'Location', 'best');
title(sprintf('Fig. 7: Control Amplitude - h = %.4f (with disturbance)', h));

%% ========== Performance Metrics Comparison ==========
fprintf('\n========== Performance Comparison Results (with disturbance) ==========\n');
print_metrics('h = 0.001', x_ista, u_ista, x_esta, u_esta, h, T_end);

%% ========== Helper Function: Main Simulation Loop ==========
function [t_log, x_ista, u_ista, nu_ista, ek_ista, x_esta, u_esta, nu_esta, ek_esta] = ...
    run_simulation(h, T_end, x_0, nu_0, phi_0, lambda_1, lambda_2, disturbance)

    N = floor(T_end / h);
    t_log = (0:N-1) * h;

    % ISTA data
    x_ista = zeros(1, N);
    u_ista = zeros(1, N);
    nu_ista = zeros(1, N);
    ek_ista = zeros(1, N);

    % ESTA data
    x_esta = zeros(1, N);
    u_esta = zeros(1, N);
    nu_esta = zeros(1, N);
    ek_esta = zeros(1, N);

    %% ISTA Simulation
    x_k = x_0;
    nu_k = nu_0;
    phi_k = phi_0;

    for k = 1:N
        t_k = (k - 1) * h;
        Delta_k = disturbance(t_k);

        [u_k, nu_k_plus_1, ~] = ISTA(x_k, nu_k, lambda_1, lambda_2, h);

        x_k_plus_1 = x_k + h * (u_k + phi_k);
        phi_k_plus_1 = phi_k + h * Delta_k;

        x_ista(k) = x_k_plus_1;
        u_ista(k) = u_k;
        nu_ista(k) = nu_k;
        ek_ista(k) = abs(x_k_plus_1) + nu_k^2;  % Regulation error: e_k = |x| + ν²

        x_k = x_k_plus_1;
        nu_k = nu_k_plus_1;
        phi_k = phi_k_plus_1;
    end

    %% ESTA Simulation
    x_k = x_0;
    nu_k = nu_0;
    phi_k = phi_0;

    for k = 1:N
        t_k = (k - 1) * h;
        Delta_k = disturbance(t_k);

        [u_k, nu_k_plus_1] = ESTA(x_k, nu_k, lambda_1, lambda_2, h);

        x_k_plus_1 = x_k + h * (u_k + phi_k);
        phi_k_plus_1 = phi_k + h * Delta_k;

        x_esta(k) = x_k_plus_1;
        u_esta(k) = u_k;
        nu_esta(k) = nu_k;
        ek_esta(k) = abs(x_k_plus_1) + nu_k^2;  % Regulation error: e_k = |x| + ν²

        x_k = x_k_plus_1;
        nu_k = nu_k_plus_1;
        phi_k = phi_k_plus_1;
    end
end

%% ========== Helper Function: Performance Metrics Printing ==========
function print_metrics(label, x_ista, u_ista, x_esta, u_esta, h, T_end)

    fprintf('\nStep size %s:\n', label);
    fprintf('Algorithm\t\tTerminal Error\t\tControl Energy\t\tPeak Control\n');
    fprintf('------------------------------------------------------------\n');

    % ISTA metrics
    final_err_ista = abs(x_ista(end));
    energy_ista = sum(u_ista.^2) * h;
    peak_ista = max(abs(u_ista));

    % ESTA metrics
    final_err_esta = abs(x_esta(end));
    energy_esta = sum(u_esta.^2) * h;
    peak_esta = max(abs(u_esta));

    fprintf('ISTA\t\t%.4e\t\t%.4f\t\t%.4f\n', ...
        final_err_ista, energy_ista, peak_ista);
    fprintf('ESTA\t\t%.4e\t\t%.4f\t\t%.4f\n', ...
        final_err_esta, energy_esta, peak_esta);
end