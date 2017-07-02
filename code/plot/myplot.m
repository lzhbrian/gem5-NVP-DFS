clear; clc; close all;

% load traverse_duty_result.txt
% load traverse_duty_result_ori.txt      
% load traverse_low_result.txt
% load traverse_low_result_ori.txt
% load traverse_high_result.txt
% load traverse_high_result_ori.txt

%% High
filename = 'traverse_high_result.txt';
[hhh, traverse_high_result] = textread(filename, '%s %f')
filename = 'traverse_high_result_ori.txt';
[hhh, traverse_high_result_ori] = textread(filename, '%s %f')
filename = 'traverse_high_result_time_count.txt';
[hhh, open_close_count, switch_freq_count] = textread(filename, '%s %d %d')
filename = 'traverse_high_result_ori_time_count.txt';
[hhh, open_close_ori_count, switch_freq_ori_count] = textread(filename, '%s %d %d')

f1 = figure();
subplot(1,2,1)
hold on;
x_axis = linspace(0, 10, 21);
x_axis = x_axis(2:21)
plot(x_axis, traverse_high_result, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori, '--*', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result + 0.1 * open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori + 0.1 * open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM', 'DFS with 0.1s per POWER op', 'TwoThresSM with 0.1s per POWER op', 'location', 'northeast')
% title('Performance')
xlabel('High level (Voltage)')
ylabel('Execution time (Second)')
set(gca, 'fontsize', 16)

subplot(1,2,2)
hold on;
plot(x_axis, open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM')
% title('# of POWERON & POWEROFF')
xlabel('High level (Voltage)')
ylabel('# of POWERON & POWEROFF')
set(gca, 'fontsize', 16)
saveas(f1, 'high')



%% Low
filename = 'traverse_low_result.txt';
[hhh, traverse_high_result] = textread(filename, '%s %f')
filename = 'traverse_low_result_ori.txt';
[hhh, traverse_high_result_ori] = textread(filename, '%s %f')
filename = 'traverse_low_result_time_count.txt';
[hhh, open_close_count, switch_freq_count] = textread(filename, '%s %d %d')
filename = 'traverse_low_result_ori_time_count.txt';
[hhh, open_close_ori_count, switch_freq_ori_count] = textread(filename, '%s %d %d')

f1 = figure();
subplot(1,2,1)
hold on;
x_axis = linspace(0, 7.5, 11);
x_axis = x_axis(2:11)
plot(x_axis, traverse_high_result, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori, '--*', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result + 0.1 * open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori + 0.1 * open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM', 'DFS with 0.1s per POWER op', 'TwoThresSM with 0.1s per POWER op', 'location', 'northeast')
% title('Performance')
xlabel('Low level (Voltage)')
ylabel('Execution time (Second)')
ylim([0 10])
set(gca, 'fontsize', 16)

subplot(1,2,2)
hold on;
plot(x_axis, open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM')
% title('# of POWERON & POWEROFF')
xlabel('Low level (Voltage)')
ylabel('# of POWERON & POWEROFF')
set(gca, 'fontsize', 16)
saveas(f1, 'low')



%% duty ratio
filename = 'traverse_duty_result.txt';
[hhh, traverse_high_result] = textread(filename, '%s %f')
filename = 'traverse_duty_result_ori.txt';
[hhh, traverse_high_result_ori] = textread(filename, '%s %f')
filename = 'traverse_duty_result_time_count.txt';
[hhh, open_close_count, switch_freq_count] = textread(filename, '%s %d %d')
filename = 'traverse_duty_result_ori_time_count.txt';
[hhh, open_close_ori_count, switch_freq_ori_count] = textread(filename, '%s %d %d')

f1 = figure();
subplot(1,2,1)
hold on;
x_axis = linspace(0, 1, 11);
plot(x_axis, traverse_high_result, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori, '--*', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result + 0.1 * open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, traverse_high_result_ori + 0.1 * open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM', 'DFS with 0.1s per POWER op', 'TwoThresSM with 0.1s per POWER op', 'location', 'northeast')
% title('Performance')
xlabel('Duty ratio')
ylabel('Execution time (Second)')
ylim([0 10])
set(gca, 'fontsize', 16)

subplot(1,2,2)
hold on;
plot(x_axis, open_close_count, '-^', 'LineWidth', 3, 'MarkerSize', 8)
plot(x_axis, open_close_ori_count, '--*', 'LineWidth', 3, 'MarkerSize', 8)
legend('DFS', 'TwoThresSM')
% title('# of POWERON & POWEROFF')
xlabel('Duty ratio')
ylabel('# of POWERON & POWEROFF')
set(gca, 'fontsize', 16)
saveas(f1, 'duty')


