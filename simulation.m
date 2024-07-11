% Script to visualize movement of platofrm

start_time = 0;
end_time = 2*pi;
time_steps = 120;
time = linspace(start_time, end_time, time_steps);

for t = time
    % Angles for this rotation
    y_r = 2*pi/32 * sin(t); % asin(wx)
    x_r = 2*pi/32 * cos(t); % asin(-wy/cos(y_r)) % perhaps xr and yr need to be swapped

    z_r = atan(-(sin(x_r) * sin(y_r)) / (cos(x_r) * cos(y_r)));

    % Rotation matrices
    R = [cos(y_r) * cos(z_r), -cos(y_r) * sin(z_r), sin(y_r);
         sin(x_r) * sin(y_r) * cos(z_r) + cos(x_r) * sin(z_r), cos(x_r) * cos(z_r) - sin(x_r) * sin(y_r) * sin(z_r), -sin(x_r) * cos(y_r);
         sin(x_r) * sin(z_r) - cos(x_r) * sin(y_r) * cos(z_r), sin(x_r) * cos(z_r) + cos(x_r) * sin(y_r) * sin(z_r), cos(x_r) * cos(y_r)];
    R_s = [R(1,1), R(1,2), R(1,3); R(2,1), R(2,2), R(2,3); R(3,1), R(3,2), R(3,3)];

    % Inverse kinematics:
    b = 0.075; % Distance from the center of the platform to the spherical joint [m]
    l1 = 0.09; % Length of the first link [m];
    l2 = 0.18; % Length of the second link [m]; 
    h = 0.22; % Height of the platform
    p = 0.15; % Distance from the center of the manipulator to the joint
    alpha = [0, 2*pi/3, 4*pi/3]; % Rotation angles for each arm

    % Vectors for each arm at the joints
    points = zeros(3, 3); % Initialize matrix to store points
    basepoints = zeros(3, 3);

    for i = 1:3
        O0O7j_vector2 = [0; 0; h] + (R_s * rotz(rad2deg(alpha(i))) * [p; 0; 0]); % Independent of middle joints 
        points(i, :) = O0O7j_vector2'; % [x1, y1, z1; x2, y2, z2; ...]
        base = [0; 0; 0] + (eye(3) * rotz(rad2deg(alpha(i))) * [b; 0; 0]);
        basepoints(i, :) = base';
    end

    for k = 1:3
        A(k) = 2 * l1 * cos(alpha(k)) * (-points(k, 1) + b * cos(alpha(k)));
        B(k) = 2 * l1 * points(k, 3) * cos(alpha(k))^2;
        C(k) = points(k, 1)^2 - 2 * b * points(k, 1) * cos(alpha(k)) + cos(alpha(k))^2 * (b^2 + l1^2 - l2^2 + points(k, 3)^2);
        theta_calc = 2 * atan2(-B(k) + sqrt(A(k)^2 + B(k)^2 - C(k)^2), C(k) - A(k)) / (2*pi) * 360;
        thetavec(k) = theta_calc;
    end

    for h = 1:3
        O0O7j_vector1 = [0; 0; 0] + rotz(rad2deg(alpha(h))) * [b; 0; 0] + rotz(rad2deg(alpha(h))) * roty(thetavec(h)) * [l1; 0; 0];
        midpoints(h, :) = O0O7j_vector1';
    end

    plotPointsMatrixIn3D(points, basepoints);
    points_3x3_1 = points;
    points_3x3_2 = basepoints;

    % Create triangles connecting points for the first triangle
    fill3(points_3x3_1(:, 1), points_3x3_1(:, 2), points_3x3_1(:, 3), [0.8500 0.3250 0.0980], 'LineWidth', 2);
    fill3([points_3x3_1(1, 1), points_3x3_1(2, 1)], [points_3x3_1(1, 2), points_3x3_1(2, 2)], [points_3x3_1(1, 3), points_3x3_1(2, 3)], [0.8500 0.3250 0.0980], 'LineWidth', 2);
    fill3([points_3x3_1(2, 1), points_3x3_1(3, 1)], [points_3x3_1(2, 2), points_3x3_1(3, 2)], [points_3x3_1(2, 3), points_3x3_1(3, 3)], [0.8500 0.3250 0.0980], 'LineWidth', 2);
    fill3([points_3x3_1(3, 1), points_3x3_1(1, 1)], [points_3x3_1(3, 2), points_3x3_1(1, 2)], [points_3x3_1(3, 3), points_3x3_1(1, 3)], [0.8500 0.3250 0.0980], 'LineWidth', 2);

    % Create triangles connecting points for the second triangle
    fill3(points_3x3_2(:, 1), points_3x3_2(:, 2), points_3x3_2(:, 3), [0 0.4470 0.7410], 'LineWidth', 2);
    fill3([points_3x3_2(1, 1), points_3x3_2(2, 1)], [points_3x3_2(1, 2), points_3x3_2(2, 2)], [points_3x3_2(1, 3), points_3x3_2(2, 3)], [0 0.4470 0.7410], 'LineWidth', 2);
    fill3([points_3x3_2(2, 1), points_3x3_2(3, 1)], [points_3x3_2(2, 2), points_3x3_2(3, 2)], [points_3x3_2(2, 3), points_3x3_2(3, 3)], [0 0.4470 0.7410], 'LineWidth', 2);
    fill3([points_3x3_2(3, 1), points_3x3_2(1, 1)], [points_3x3_2(3, 2), points_3x3_2(1, 2)], [points_3x3_2(3, 3), points_3x3_2(1, 3)], [0 0.4470 0.7410], 'LineWidth', 2);

    % Connect the vertices of the triangles
    plot3([points_3x3_1(1, 1), midpoints(1, 1)], [points_3x3_1(1, 2), midpoints(1, 2)], [points_3x3_1(1, 3), midpoints(1, 3)], 'k', 'LineWidth', 3);
    plot3([points_3x3_1(2, 1), midpoints(2, 1)], [points_3x3_1(2, 2), midpoints(2, 2)], [points_3x3_1(2, 3), midpoints(2, 3)], 'k', 'LineWidth', 3);
    plot3([points_3x3_1(3, 1), midpoints(3, 1)], [points_3x3_1(3, 2), midpoints(3, 2)], [points_3x3_1(3, 3), midpoints(3, 3)], 'k', 'LineWidth', 3);

    plot3([points_3x3_2(1, 1), midpoints(1, 1)], [points_3x3_2(1, 2), midpoints(1, 2)], [points_3x3_2(1, 3), midpoints(1, 3)], 'k', 'LineWidth', 3);
    plot3([points_3x3_2(2, 1), midpoints(2, 1)], [points_3x3_2(2, 2), midpoints(2, 2)], [points_3x3_2(2, 3), midpoints(2, 3)], 'k', 'LineWidth', 3);
    plot3([points_3x3_2(3, 1), midpoints(3, 1)], [points_3x3_2(3, 2), midpoints(3, 2)], [points_3x3_2(3, 3), midpoints(3, 3)], 'k', 'LineWidth', 3);

    % Add points on the plot for both triangles
    scatter3(points_3x3_1(:, 1), points_3x3_1(:, 2), points_3x3_1(:, 3), 'filled', 'MarkerFaceColor', 'r');
    scatter3(points_3x3_2(:, 1), points_3x3_2(:, 2), points_3x3_2(:, 3), 'filled', 'MarkerFaceColor', 'r');
    scatter3(midpoints(:, 1), midpoints(:, 2), midpoints(:, 3), 'filled', 'MarkerFaceColor', 'r');

    % Optionally: Set title and axis labels
    % title('Manipulator 3RRS initial position');
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');
    xlim([-0.2 0.2])
    ylim([-0.2 0.2])
    zlim([0 0.25])
    % Optionally: Set 3D view
    view(3);
    view(-10, 10);
    drawnow
    clf
end

function plotPointsMatrixIn3D(pointsMatrix, pointsMatrix2)
    % pointsMatrix - 3x3 matrix containing coordinates of three points in format [x1, y1, z1; x2, y2, z2; x3, y3, z3]

    % Create new 3D plot window
    % figure;
    grid on; hold on; axis equal;

    % Plot points on the graph
    plot3(pointsMatrix(:, 1), pointsMatrix(:, 2), pointsMatrix(:, 3), 'k', 'LineWidth', 1.5);
    plot3(pointsMatrix(:, 1), pointsMatrix(:, 2), pointsMatrix(:, 3), 'k', 'LineWidth', 1.5);

    scatter3(pointsMatrix2(:, 1), pointsMatrix2(:, 2), pointsMatrix2(:, 3), 'filled');
    line(pointsMatrix2(:, 1), pointsMatrix2(:, 2), pointsMatrix2(:, 3));

    % Set axis labels
    xlabel('X Axis');
    ylabel('Y Axis');
    zlabel('Z Axis');

    % Set 3D view
    view(3);
end
