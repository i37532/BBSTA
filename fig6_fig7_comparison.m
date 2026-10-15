%% 复现作者提供的 BBSTA 仿真（对应论文图 6 和图 7）
% 来源：../code1.png。沿用作者局部变量 u、nu；第一状态记为 x。
% nu = nu_ctrl + phi 为闭环第二状态，nu_ctrl 是原控制器积分状态，phi_dot = Delta。
% ISTA_author_step 返回 [u_k, nu_next]，随后执行 x_next = x_k + h*u_k。
% 其中 h*Delta 先加入 nu_next，再通过 u_k 进入 x_next；不另加 h*phi。
% 此处 u_k 是截图的步进量，对应此前的 f1_step，不是原对象的物理控制输入。
%
% 图 6：e_k = abs(x_k) + nu_k^2，使用同一时刻的状态值。
% 图 7：abs(u_plot)，u_plot = nu-lambda_1*sqrt(abs(x))*sign(x)。
% 隐式方法的 u_plot 一般不等于 u_k，两者分别保存。
% 对数绘图仅将临时数组中的精确零设为 NaN，不改变原始数据。
%
% 原 ISTA.m、ESTA.m 和 fig6_fig7_comparison_before_author_fix.m 保留原样。
% 结果保存到 ../output/author_reproduction/matlab/。

clear; clc; close all;

%% Parameters and initial conditions
lambda_1 = 10;
lambda_2 = 6;
T_end = 5;
h = 0.001;
x_0 = 10;
nu_ctrl_0 = 10;
phi_0 = 0;
nu_0 = nu_ctrl_0 + phi_0;
disturbance = @(t) 2 * (1 + sin(2 * t^2));

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
output_dir = fullfile(fileparts(script_dir), 'output', ...
    'author_reproduction', 'matlab');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

%% Simulation: N+1 state samples and N step quantities
results = run_simulation(h, T_end, x_0, nu_0, ...
    lambda_1, lambda_2, disturbance);
t = results.t;
t_step = results.t_step;
ista = results.ISTA;
esta = results.ESTA;

%% Fig. 6: regulation error from the transformed closed-loop states
fig6 = figure('Name', 'Fig. 6 - Author simulation: regulation error', ...
    'Color', 'w', 'Position', [50 50 800 600]);
semilogy(t, positive_for_log(esta.ek), 'r-', 'LineWidth', 1.2);
hold on;
semilogy(t, positive_for_log(ista.ek), 'b--', 'LineWidth', 1.2);
grid on;
xlim([0 T_end]); ylim([1e-14 1e3]);
xlabel('Time [s]'); ylabel('e_k = |x_k| + \nu_k^2');
legend('ESTA (explicit)', 'ISTA (author simulation)', 'Location', 'best');
title(sprintf('Fig. 6: author simulation, h = %.3f s', h));
save_figure(fig6, output_dir, 'fig6_regulation_error');

%% Fig. 7: author's postprocessed signal u_plot
fig7 = figure('Name', 'Fig. 7 - Author postprocessed signal', ...
    'Color', 'w', 'Position', [100 100 800 600]);
semilogy(t, positive_for_log(abs(esta.u_plot)), 'r-', 'LineWidth', 1.2);
hold on;
semilogy(t, positive_for_log(abs(ista.u_plot)), 'b--', 'LineWidth', 1.2);
grid on;
xlim([0 T_end]); ylim([1e-6 1e2]);
xlabel('Time [s]');
ylabel('|u_{plot}| = |\nu - \lambda_1 |x|^{1/2} sign(x)|');
legend('ESTA (explicit)', 'ISTA (author simulation)', 'Location', 'best');
title(sprintf('Fig. 7: author postprocessing, h = %.3f s', h));
save_figure(fig7, output_dir, 'fig7_postprocessed_amplitude');

%% Supplement: distinguish the implicit step rate from Fig. 7's signal
fig_check = figure('Name', 'ISTA: postprocessed signal versus step rate', ...
    'Color', 'w', 'Position', [150 150 900 600]);
subplot(2, 1, 1);
plot(t, ista.u_plot, 'b--', 'LineWidth', 1.1); hold on;
plot(t_step, ista.u, 'k-', 'LineWidth', 1.1);
grid on; xlim([0 T_end]);
xlabel('Time [s]'); ylabel('Signed quantity');
legend('u_{plot} (Fig. 7)', 'u_k (state update)', 'Location', 'best');
title('ISTA: distinct quantities, both retained in the exported data');
subplot(2, 1, 2);
plot(t, ista.u_plot, 'b--', 'LineWidth', 1.1); hold on;
plot(t_step, ista.u, 'k-', 'LineWidth', 1.1);
grid on; xlim([max(0, T_end - 1) T_end]);
last_state = t >= max(0, T_end - 1);
last_step = t_step >= max(0, T_end - 1);
last_values = [ista.u_plot(last_state); ista.u(last_step)];
padding = 0.05 * max(max(last_values) - min(last_values), eps);
ylim([min(last_values) - padding, max(last_values) + padding]);
xlabel('Time [s]'); ylabel('Signed quantity');
title('Last second');
save_figure(fig_check, output_dir, 'ista_quantity_check');

%% Export raw, unmodified simulation values
parameters = struct('lambda_1', lambda_1, 'lambda_2', lambda_2, ...
    'h', h, 'T_end', T_end, 'x_0', x_0, 'nu_ctrl_0', nu_ctrl_0, ...
    'phi_0', phi_0, 'nu_0', nu_0);
save(fullfile(output_dir, 'author_reproduction.mat'), 'results', 'parameters');
state_table = table(t, ista.x, ista.nu, ista.ek, ista.u_plot, ...
    esta.x, esta.nu, esta.ek, esta.u_plot, ...
    'VariableNames', {'t', 'x_ISTA', 'nu_ISTA', 'e_ISTA', 'u_plot_ISTA', ...
    'x_ESTA', 'nu_ESTA', 'e_ESTA', 'u_plot_ESTA'});
writetable(state_table, fullfile(output_dir, 'author_states.csv'));
step_table = table(t_step, results.Delta, ista.u, esta.u, ...
    ista.r, 'VariableNames', {'t', 'Delta', 'u_ISTA', ...
    'u_ESTA', 'r_ISTA'});
writetable(step_table, fullfile(output_dir, 'author_steps.csv'));

%% Metrics: terminal states at t=N*h; quantity definitions are explicit
fprintf('\nAuthor two-state simulation, terminal t = %.6f s\n', t(end));
fprintf('The second state is nu=nu_ctrl+phi; u is the author step rate.\n');
fprintf('Algorithm         x(T)             nu(T)              e(T)\n');
print_terminal('ISTA', ista);
print_terminal('ESTA', esta);
fprintf('\nSeparate signal statistics (not physical-control energy):\n');
fprintf('Algorithm     peak |u_plot|      h*sum(u.^2)\n');
print_signals('ISTA', ista, h);
print_signals('ESTA', esta, h);
fprintf('\nOutputs saved to:\n%s\n', output_dir);

%% Local functions
function results = run_simulation(h, T_end, x_0, nu_0, ...
        lambda_1, lambda_2, disturbance)
    N = floor(T_end / h);
    results.t = (0:N)' * h;
    results.t_step = (0:N-1)' * h;
    results.Delta = zeros(N, 1);
    names = {'ISTA', 'ESTA'};
    for idx = 1:numel(names)
        name = names{idx};
        results.(name).x = zeros(N + 1, 1);
        results.(name).nu = zeros(N + 1, 1);
        results.(name).u = zeros(N, 1);
        results.(name).x(1) = x_0;
        results.(name).nu(1) = nu_0;
    end
    results.ISTA.r = zeros(N, 1);

    for k = 1:N
        Delta_k = disturbance(results.t_step(k));
        results.Delta(k) = Delta_k;

        [u_k, nu_next, r_k] = ISTA_author_step( ...
            results.ISTA.x(k), results.ISTA.nu(k), ...
            lambda_1, lambda_2, h, Delta_k);
        results.ISTA.x(k + 1) = results.ISTA.x(k) + h * u_k;
        results.ISTA.nu(k + 1) = nu_next;
        results.ISTA.u(k) = u_k;
        results.ISTA.r(k) = r_k;

        % Reuse the explicit controller formula with nu as its second
        % argument to obtain the explicit CLOSED-LOOP increment. Add h*Delta
        % to the second state only after computing the current-state rate.
        [u_k, nu_nominal_next] = ESTA( ...
            results.ESTA.x(k), results.ESTA.nu(k), ...
            lambda_1, lambda_2, h);
        results.ESTA.x(k + 1) = results.ESTA.x(k) + h * u_k;
        results.ESTA.nu(k + 1) = nu_nominal_next + h * Delta_k;
        results.ESTA.u(k) = u_k;
    end

    for idx = 1:numel(names)
        name = names{idx};
        x = results.(name).x;
        nu = results.(name).nu;
        results.(name).ek = abs(x) + nu.^2;
        results.(name).u_plot = nu - lambda_1 * sqrt(abs(x)) .* sign(x);
    end
end

function y = positive_for_log(y)
    % Display only: do not alter the raw result arrays or add a floor.
    y(y <= 0) = NaN;
end

function save_figure(fig, output_dir, basename)
    print(fig, fullfile(output_dir, [basename '.png']), '-dpng', '-r180');
    savefig(fig, fullfile(output_dir, [basename '.fig']));
end

function print_terminal(name, data)
    fprintf('%-9s  % .10e  % .10e  %.10e\n', ...
        name, data.x(end), data.nu(end), data.ek(end));
end

function print_signals(name, data, h)
    fprintf('%-9s  %.10e       %.10e\n', ...
        name, max(abs(data.u_plot)), h * sum(data.u.^2));
end
