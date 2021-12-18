function plotSimulationResults6DoF(simulation)
% Plots the results of a 6DoF drone simulation. The struct simulation is
% obtained through the function simulateDrone6DoF().

format = 'png'; % for Word users
% format = 'eps'; % for LaTeX users

figure;
hold on;
plot3(simulation.Xg.signals.values(:, 1), simulation.Xg.signals.values(:, 2),...
    simulation.Xg.signals.values(:, 3), 'LineWidth', 2);
xlabel('X (m)', 'FontSize', 14);
ylabel('Y (m)', 'FontSize', 14);
zlabel('Z (m)', 'FontSize', 14);
set(gca, 'FontSize', 14);
grid on;
axis equal;
view(-45, 30);
savePlot(sprintf('x_z_%c', simulation.experiment), format);

figure;
subplot(3, 1, 1);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 1), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 1), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('X (m)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('x_r', 'x');
grid on;

subplot(3, 1, 2);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 2), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('y_r', 'y');
grid on;

subplot(3, 1, 3);
plot(simulation.Xr.time, simulation.Xr.signals.values(:, 3), 'r', 'LineWidth', 2);
hold on;
plot(simulation.Xg.time, simulation.Xg.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('Z (m)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('z_r', 'z');
grid on;
savePlot(sprintf('t_Xg_%c', simulation.experiment), format);

figure;
subplot(3, 1, 1);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 1), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 1), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('\phi (rad)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('\phi_r', '\phi');
grid on;

subplot(3, 1, 2);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 2), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('\theta (rad)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('\theta_r', '\theta');
grid on;

subplot(3, 1, 3);
plot(simulation.eulerr.time, simulation.eulerr.signals.values(:, 3), 'r', 'LineWidth', 2);
hold on;
plot(simulation.euler.time, simulation.euler.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 12);
ylabel('\psi (rad)', 'FontSize', 12);
set(gca, 'FontSize', 12);
legend('\psi_r', '\psi');
grid on;
savePlot(sprintf('t_euler_%c', simulation.experiment), format);

figure;
subplot(2, 1, 1);
plot(simulation.Vg.time, simulation.Vg.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14);
ylabel('V_g (m/s)', 'FontSize', 14);
set(gca, 'FontSize', 14);
legend('v_x', 'v_y', 'v_z');
grid on;

subplot(2, 1, 2);
plot(simulation.wb.time, simulation.wb.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14);
ylabel('\omega_b (rad/s)', 'FontSize', 14);
set(gca, 'FontSize', 14);
legend('p', 'q', 'r');
grid on;
savePlot(sprintf('t_Vg_%c', simulation.experiment), format);

figure;
subplot(2, 1, 1);
plot(simulation.f.time, simulation.f.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14);
ylabel('f (N)', 'FontSize', 14);
legend();
set(gca, 'FontSize', 14);
grid on;

subplot(2, 1, 2);
plot(simulation.Mxyz.time, simulation.Mxyz.signals.values, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 14);
ylabel('\tau (N m)', 'FontSize', 14);
set(gca, 'FontSize', 14);
legend('\tau_x', '\tau_y', '\tau_z');
grid on;
savePlot(sprintf('t_f_tau_%c', simulation.experiment), format);

end