clc; clear; close all;

addpath(genpath(pwd));

arm = build_robot();

q_start = [0.1 0.1 0.1];
target_pos = [0.5; 0.8];

[q_final, err] = ik_gradient_descent(arm, q_start, target_pos);

[t, Q, V, A, JERK] = quintic_trajectory(q_start, q_final, 2.0, 0.01);

% Compute joint torques
TAU = compute_torques(arm, Q, V, A);


animate_robot(arm, Q, target_pos);
plot_cartesian_path(arm, Q, target_pos);
plot_physics(t, V, A, JERK);

figure;
plot(err); grid on;
title('Gradient Descent Convergence');
plot_torques(t, TAU);

