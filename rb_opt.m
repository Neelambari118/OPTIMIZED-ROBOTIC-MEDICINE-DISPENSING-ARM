
%   Optimized Robotic Medicine-Dispensing Arm 

clear; clc; close all;

% CONFIGURATION & TARGETS

% Robot Geometry
rob.a1 = 0.35;       %Shoulder to Elbow
rob.a2 = 0.30;        %Elbow to Wrist
rob.d0 = 0.18;       %base height

rob.g = 9.81; 
rob.m_links = [2.0, 1.5, 0.5]; 
rob.m_payload = 0.2;            


PICK_TARGET = [ 0.6,  0.0, 0.00,  0.0];         %[X, Y, Z, Angle]
DROP_TARGET = [0.0,  0.5, 0.5, 90.0];


%master function

[Q, V, A, Tau, Mom, ObjPos] = generate_path(rob, PICK_TARGET, DROP_TARGET);

if isempty(Q)
    error('Simulation stopped. Optimization failed to converge.');
end

% PLOT 1: DYNAMICS (Torque & Momentum)

figure('Name', 'Physics Data', 'Color', 'w');
subplot(3,1,1);
plot(Tau(:,1), 'LineWidth', 1.5); hold on;
plot(Tau(:,2), 'LineWidth', 1.5);
plot(Tau(:,3), 'LineWidth', 1.5, 'LineStyle', '--');
legend('Shoulder Torque', 'Elbow Torque', 'Z-Force (Gravity)');
title('Calculated Torque & Force (Optimization Results)'); grid on; ylabel('Nm / N');

subplot(3,1,2);
plot(Mom(:,1), 'LineWidth', 1.5); hold on;
plot(Mom(:,2), 'LineWidth', 1.5);
legend('Link 1 Momentum', 'Link 2 Momentum');
title('Calculated Angular Momentum'); grid on; ylabel('kg m^2/s');

subplot(3,1,3);
plot(V(:,1), 'LineWidth', 1.5); hold on;
plot(V(:,2), 'LineWidth', 1.5);
plot(V(:,3), 'LineWidth', 1.5);
plot(V(:,4), 'LineWidth', 1.5, 'LineStyle', ':'); % Added q4 (Theta 4)
legend('Theta1 Vel', 'Theta2 Vel', 'd3 Vel', 'Theta4 Vel');
title('Joint Velocities'); grid on; ylabel('rad/s');


% 3D ANIMATION

figure('Name', 'SCARA 3D Simulation (Gradient Descent)', 'Color', 'w');
axis equal; axis([-0.7 0.7 -0.7 0.7 0 0.6]);
grid on; view(3); hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('SCARA Path with Gradient Descent IK Control');

%Environment
plot3(PICK_TARGET(1), PICK_TARGET(2), PICK_TARGET(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot3(DROP_TARGET(1), DROP_TARGET(2), DROP_TARGET(3), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

% Body
h_arm = plot3([0 0], [0 0], [0 0], 'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
h_obj = plot3(0, 0, 0, 's', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'Color', 'k');

% Animate
steps = size(Q, 1);
for i = 1:steps
    q = Q(i, :);
    obj_p = ObjPos(i, :);
    
    % Forward Kinematics for Link Positions
    p0 = [0, 0, rob.d0];
    p1 = [rob.a1*cos(q(1)), rob.a1*sin(q(1)), rob.d0];
    p2 = [p1(1) + rob.a2*cos(q(1)+q(2)), p1(2) + rob.a2*sin(q(1)+q(2)), rob.d0];
    p3 = [p2(1), p2(2), rob.d0 - q(3)];
    
    % Update Graph
    set(h_arm, 'XData', [p0(1) p1(1) p2(1) p3(1)], 'YData', [p0(2) p1(2) p2(2) p3(2)], 'ZData', [p0(3) p1(3) p2(3) p3(3)]);
    set(h_obj, 'XData', obj_p(1), 'YData', obj_p(2), 'ZData', obj_p(3));
    
    drawnow limitrate;
    pause(0.01);
end

%  GRADIENT DESCENT OPTIMIZATION

function q_opt = inverse_kinematics_gradient(rob, target, q_guess)
    % Optimization Hyperparameters
    alpha = 0.05;      % Learning rate
    tolerance = 1e-5;  % Error threshold
    max_iter = 1000;   % Max iterations
    
    target_pos = target(1:3); % [x, y, z]
    q = q_guess;              % Initial seed
    
    for iter = 1:max_iter
        % 1. Forward Kinematics (Current Position)
        x_c = rob.a1*cos(q(1)) + rob.a2*cos(q(1)+q(2));
        y_c = rob.a1*sin(q(1)) + rob.a2*sin(q(1)+q(2));
        z_c = rob.d0 - q(3);
        curr_pos = [x_c, y_c, z_c];
        
        % 2. Error Vector
        error = target_pos - curr_pos;
        if norm(error) < tolerance, break; end
        
        % 3. Jacobian Matrix Calculation
        % Derivatives of [x; y; z] with respect to [theta1; theta2; d3]
        J = [(-rob.a1*sin(q(1)) - rob.a2*sin(q(1)+q(2))), (-rob.a2*sin(q(1)+q(2))), 0;
              (rob.a1*cos(q(1)) + rob.a2*cos(q(1)+q(2))),  (rob.a2*cos(q(1)+q(2))), 0;
              0, 0, -1];
        
        % 4. Gradient Descent Step
        % DeltaQ = LearningRate * JacobianTranspose * Error
        delta_q = alpha * (J' * error');
        q(1:3) = q(1:3) + delta_q';
    end
    
    % Compute orientation theta4
    theta4 = deg2rad(target(4)) - (q(1) + q(2));
    q_opt = [q(1), q(2), q(3), theta4];
end

% ==============================================================================
%  SUPPORTING FUNCTIONS (Dynamics & Path Generation)
% ==============================================================================
function [tau, L] = get_dynamics(rob, q, v, a, holding)
    m_load = 0;
    if holding, m_load = rob.m_payload; end
    
    % Gravity on prismatic joint (Z-axis)
    force_z = (rob.m_links(3) + m_load) * (a(3) + rob.g);
    
    % Inertia approximation for rotational links
    I2 = (rob.m_links(2) + rob.m_links(3) + m_load) * rob.a2^2;
    I1 = rob.m_links(1) * rob.a1^2 + I2;
    
    tau2 = I2 * (a(1) + a(2));
    tau1 = I1 * a(1) + tau2;
    
    % Angular Momentum
    L1 = I1 * v(1);
    L2 = I2 * (v(1) + v(2));
    
    tau = [tau1, tau2, force_z];
    L = [L1, L2];
end

function [Q, V, A, Tau, Mom, ObjPos] = generate_path(rob, pick, drop)
    safe_z = 0.05; 
    waypoints = [
        0.3, 0.1, safe_z, 0, 1.0, 0;          
        pick(1), pick(2), safe_z, pick(4), 1.0, 0; 
        pick(1), pick(2), pick(3), pick(4), 1.0, 0;
        pick(1), pick(2), pick(3), pick(4), 0.5, 1;
        pick(1), pick(2), safe_z, pick(4), 1.0, 1; 
        drop(1), drop(2), safe_z, drop(4), 1.5, 1; 
        drop(1), drop(2), drop(3), drop(4), 1.0, 1;
        drop(1), drop(2), drop(3), drop(4), 0.5, 0;
        drop(1), drop(2), safe_z, drop(4), 1.0, 0; 
    ];
    
    Q=[]; V=[]; A=[]; Tau=[]; Mom=[]; ObjPos=[];
    curr_q = [0.1, 0.1, 0.1, 0]; % Initial guess
    
    for w = 1:size(waypoints, 1)
        % OPTIMIZATION STEP: Solve IK using Gradient Descent
        target_q = inverse_kinematics_gradient(rob, waypoints(w,1:4), curr_q);
        
        steps = round(waypoints(w,5) * 30);
        for i = 1:steps
            s = i/steps; 
            s_poly = 3*s^2 - 2*s^3; % Smooth quintic-style motion
            v_poly = (6*s - 6*s^2)/waypoints(w,5); 
            a_poly = (6 - 12*s)/waypoints(w,5)^2;
            
            q_now = curr_q + (target_q - curr_q) * s_poly;
            v_now = (target_q - curr_q) * v_poly;
            a_now = (target_q - curr_q) * a_poly;
            
            [tau_now, mom_now] = get_dynamics(rob, q_now, v_now, a_now, waypoints(w,6));
            
            Q=[Q; q_now]; V=[V; v_now]; A=[A; a_now]; Tau=[Tau; tau_now]; Mom=[Mom; mom_now];
            
            % Object position logic for animation
            if waypoints(w,6)
                tx=rob.a1*cos(q_now(1)) + rob.a2*cos(q_now(1)+q_now(2));
                ty=rob.a1*sin(q_now(1)) + rob.a2*sin(q_now(1)+q_now(2));
                tz=rob.d0 - q_now(3);
                ObjPos=[ObjPos; tx, ty, tz];
            else
                if w < 5, ObjPos=[ObjPos; pick(1:3)]; else, ObjPos=[ObjPos; drop(1:3)]; end
            end
        end
        curr_q = target_q;
    end
end