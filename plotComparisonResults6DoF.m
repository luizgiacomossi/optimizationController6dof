function plotComparisonResults6DoF(simulationPre, simulationPos)
% Plots the results of a 6DoF drone simulationPre. The struct simulationPre is
% obtained through the function simulateDrone6DoF().

format = 'png'; % for Word users
% format = 'eps'; % for LaTeX users

figure;
hold on;
plot3(simulationPre.Xg.signals.values(:, 1), simulationPre.Xg.signals.values(:, 2),...
    simulationPre.Xg.signals.values(:, 3), 'LineWidth', 2);
xlabel('X (m)', 'FontSize', 20);
ylabel('Y (m)', 'FontSize', 20);
zlabel('Z (m)', 'FontSize', 20);
set(gca, 'FontSize', 20);
grid on;
axis equal;
view(-29.483981618372724,57.088091985796858);
hold on;
plot3(simulationPos.Xg.signals.values(:, 1), simulationPos.Xg.signals.values(:, 2),...
    simulationPos.Xg.signals.values(:, 3), 'LineWidth', 2);
hold on;
plot3(simulationPos.Xr.signals.values(:, 1), simulationPos.Xr.signals.values(:, 2),...
    simulationPos.Xr.signals.values(:, 3),'--', 'LineWidth', 1, 'Color', 'black');
legend( 'Pre Optimization','Post Optimization','Reference');
savePlot(sprintf('comp_traj_%c', simulationPre.experiment), format);


figure;
subplot(4, 1, 1);
plot(simulationPre.Xr.time, simulationPre.Xr.signals.values(:, 1), '--','Color' ,'black', 'LineWidth', 1);
hold on;
plot(simulationPre.Xg.time, simulationPre.Xg.signals.values(:, 1), 'Color','b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 18);
ylabel('X (m)', 'FontSize',18);
set(gca, 'FontSize', 12);
grid on;
hold on;
plot(simulationPos.Xg.time, simulationPos.Xg.signals.values(:, 1), 'color','#D95319','LineWidth', 2);
legend('x_r', 'xPre','xPost' );

subplot(4, 1, 2);
plot(simulationPre.Xr.time, simulationPre.Xr.signals.values(:, 2), '--','Color' ,'black', 'LineWidth', 1);
hold on;
plot(simulationPre.Xg.time, simulationPre.Xg.signals.values(:, 2), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 18);
ylabel('Y (m)', 'FontSize', 18);
set(gca, 'FontSize', 12);
grid on;
hold on;
plot(simulationPos.Xg.time, simulationPos.Xg.signals.values(:, 2), 'color','#D95319', 'LineWidth', 2);
legend('y_r', 'yPre','yPost');


subplot(4, 1, 3);
plot(simulationPre.Xr.time, simulationPre.Xr.signals.values(:, 3),  '--','Color' ,'black', 'LineWidth', 1);
hold on;
plot(simulationPre.Xg.time, simulationPre.Xg.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 18);
ylabel('Z (m)', 'FontSize', 18);
set(gca, 'FontSize', 12);
grid on;
plot(simulationPos.Xg.time, simulationPos.Xg.signals.values(:, 3), 'color','#D95319', 'LineWidth', 2);
legend('z_r', 'zPre', 'zPost');

subplot(4, 1, 4);
plot(simulationPre.eulerr.time, simulationPre.eulerr.signals.values(:, 3), '--','Color' ,'black', 'LineWidth', 1);
hold on;
plot(simulationPre.euler.time, simulationPre.euler.signals.values(:, 3), 'b', 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 18);
ylabel('\psi (rad)', 'FontSize', 18);
set(gca, 'FontSize', 16);
grid on;
plot(simulationPos.euler.time, simulationPos.euler.signals.values(:, 3),  'color','#D95319', 'LineWidth', 2);
legend('\psi_r', '\psi_{Pre}', '\psi_{Post}');

savePlot(sprintf('comp_Xg_%c', simulationPre.experiment), format);


end