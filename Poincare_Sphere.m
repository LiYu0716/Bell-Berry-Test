clear all;
close all;
%%
% 1. Define Phase Retardation Functions using 2x2 Jones Matrices
% theta: fast axis angle (rad), delta: phase retardance (rad)
jones_retarder = @(theta, delta) ...
    exp(-1i*delta/2)*[cos(theta)^2 + exp(1i*delta)*sin(theta)^2, (1-exp(1i*delta))*sin(theta)*cos(theta); ...
     (1-exp(1i*delta))*sin(theta)*cos(theta), sin(theta)^2 + exp(1i*delta)*cos(theta)^2];

% 2. Define Waveplate Sequence Physical Parameters
wp_angles = deg2rad([45, 90, 67.5, 0]); % Fast axis angles for QWP1, HWP1, QWP2, HWP2
% HQHQ: [45, 22.5, 0, 67.5]/[22.5, 33.75, 45, 11.25]/[30, 75, 30, 75]/[45, 67.5, 0, 67.5]
% [45, 45, 45, 0]/[45, 90, 45, 0]
% QQH: [45, 90, 67.5, 0]/[45, 0, 22.5, 0]/[30, 30, 30, 0]
wp_delays = [pi/2, pi/2, pi, pi];      % Target retardations (QWP = pi/2, HWP = pi)

% 3. Initialize State with a 2-by-1 Jones Vector (Horizontal Linear Polarization)
J_start = [1; 0]; 
J_current_base = J_start;

% 4. Animation Settings
res = 100; % Animation steps per waveplate
total_frames = 4 * res;
trajectory = zeros(total_frames, 3); % Array to hold converted 3D coordinates
labels = {'QWP1', 'HWP1', 'QWP2', 'HWP2'};
label_coords = zeros(4, 3);

% 5. Compute Moving States using 2x1 Vectors
frame = 1;
for k = 1:4
    theta = wp_angles(k);
    max_delta = wp_delays(k);
    
    for step = 1:res
        % Dynamically increase waveplate retardance from 0 to max
        current_delta = max_delta * (step / res);
        
        % Generate the 2x2 operator matrix
        M_waveplate = jones_retarder(theta, current_delta);
        
        % Core Operation: Transform the 2x1 Input Vector
        J_moving = M_waveplate * J_current_base; 
        
        % Convert 2x1 Jones Vector into 3D Poincaré Mapping Coordinates
        Ex = J_moving(1); 
        Ey = J_moving(2);
        I  = conj(Ex)*Ex + conj(Ey)*Ey; % Total intensity
        
        % Store 3D spatial points calculated directly from 2x1 complex vector entries
        trajectory(frame, 1) = real((conj(Ex)*Ex - conj(Ey)*Ey) / I); % S1 axis
        trajectory(frame, 2) = real((conj(Ex)*Ey + Ex*conj(Ey)) / I); % S2 axis
        trajectory(frame, 3) = real(1i*(conj(Ex)*Ey - Ex*conj(Ey)) / I); % S3 axis
        
        frame = frame + 1;
    end
    
    % Update the base 2x1 vector state using the fully completed waveplate transformation
    M_full = jones_retarder(theta, max_delta);
    J_current_base = M_full * J_current_base;
    
    % Save label location at the end of each waveplate segment
    label_coords(k, :) = trajectory(frame-1, :);
end

% 6. Plot the Poincaré Sphere Canvas
[X, Y, Z] = sphere(60);
figure('Color', 'w');
mesh(X, Y, Z, 'EdgeColor', [0.85 0.85 0.85], 'FaceAlpha', 0.02);
hold on; axis equal; grid on;
view(125, 20);
xlabel('S_1 (H / V)'); ylabel('S_2 (D / A)'); zlabel('S_3 (R / L)');
title('Pooncé Sphere: 2\times1 Jones Vector Loop Animation');


% % 7. Draw the Start Point Anchor (H)
% plot3(1, 0, 0, 'ks', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
% text(1.1, 0, 0, 'START (H)', 'Color', 'g', 'FontSize', 11, 'FontWeight', 'bold');
% 
% % 8. Initialize Moving Graphics Objects
% path_line = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2.5);
% vector_tip = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
% 
% % 9. Execute Live Motion Render
% for f = 1:total_frames
%     % Update line history and leading point tracking
%     set(path_line, 'XData', trajectory(1:f, 1), 'YData', trajectory(1:f, 2), 'ZData', trajectory(1:f, 3));
%     set(vector_tip, 'XData', trajectory(f, 1), 'YData', trajectory(f, 2), 'ZData', trajectory(f, 3));
% 
%     % Drop text markers dynamically when a component stage calculation completes
%     if mod(f, res) == 0
%         stage = f / res;
%         plot3(label_coords(stage, 1), label_coords(stage, 2), label_coords(stage, 3), 'ro', 'MarkerFaceColor', 'r');
%         text(label_coords(stage, 1)*1.1, label_coords(stage, 2)*1.1, label_coords(stage, 3)*1.1, ...
%              labels{stage}, 'FontSize', 10, 'FontWeight', 'bold');
%     end
% 
%     drawnow;
%     pause(0.02);
% end


color_indices = kron(1:4, ones(1, res))'; 

colormap([0 0 0; 1 0 0;0 1 0; 0 0 1; 1 1 0; 1 0 0]);

% 8. Initialize Moving Graphics Objects

path_line = surface([NaN NaN; NaN NaN], [NaN NaN; NaN NaN], [NaN NaN; NaN NaN], [NaN NaN; NaN NaN], ...
                    'EdgeColor', 'flat', 'FaceColor', 'none', 'LineWidth', 3);
vector_tip = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 7);

% 9. Execute Live Motion Render
for f = 1:total_frames
    
    x_data = [trajectory(1:f, 1), trajectory(1:f, 1)];
    y_data = [trajectory(1:f, 2), trajectory(1:f, 2)];
    z_data = [trajectory(1:f, 3), trajectory(1:f, 3)];
    c_data = [color_indices(1:f), color_indices(1:f)];
    
    set(path_line, 'XData', x_data, 'YData', y_data, 'ZData', z_data, 'CData', c_data);
    set(vector_tip, 'XData', trajectory(f, 1), 'YData', trajectory(f, 2), 'ZData', trajectory(f, 3));
    
    if mod(f, res) == 0
        stage = f / res;
        plot3(label_coords(stage, 1), label_coords(stage, 2), label_coords(stage, 3), 'ro', 'MarkerFaceColor', 'r');
        text(label_coords(stage, 1)*1.1, label_coords(stage, 2)*1.1, label_coords(stage, 3)*1.1, ...
             labels{stage}, 'FontSize', 10, 'FontWeight', 'bold');
    end
    drawnow;
    pause(0.02);
end

%%
% =========================================================================
% Direct Numerical Line Integration via Spherical Green's Theorem
% Inputs: 'trajectory' (N x 3 matrix containing all sequential S1, S2, S3 coordinates)
% =========================================================================

numerical_area_sum = 0;
num_pts = size(trajectory, 1);

% Loop through every single discrete coordinate on the path
for i = 1:num_pts
    % Get current point and the next adjacent point along the loop
    p_curr = trajectory(i, :);
    p_next = trajectory(mod(i, num_pts) + 1, :); % Automatically wraps around to start point
    
    % Apply standard stereographic coordinate line integral projection
    % This handles curved small-circle paths without any multi-angle geometric errors
    denom = 1 + p_curr(3); % Projection denominator based on S3 pole (1 + S3)
   

    if denom > 1e-6
        % Compute signed differential segment area
        differential_area = (p_curr(1)*p_next(2) - p_curr(2)*p_next(1)) / denom;
        numerical_area_sum = numerical_area_sum + differential_area;
        
    end

end

% The final exact physical solid angle (enclosed area)
Omega = abs(numerical_area_sum);

% Print the numerical result directly to the Command Window
fprintf('\n==================================================\n');
fprintf('Direct Numerical Trajectory Area Integration:\n');
fprintf('  Enclosed Solid Angle (Omega) = %.4f steradians\n', Omega);
fprintf('  Sphere Surface Coverage      = %.2f%%\n', (Omega / (4*pi)) * 100);
fprintf('  Berry Phase                  = %.2f radians [Omega / 2]\n', (Omega / 2));
fprintf('==================================================\n');


