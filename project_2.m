data_struct = load("ESE446SimDataSet1.mat");

data = data_struct.ESE446SimDataSet1;


% Initialize the Figure
figure('Name', 'Stanford Arm Kinematics', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

% Loop through the time series data
for i = 1:size(data, 1)
    
    % Extract the 6 joint inputs for this specific time step
    q_current = data(i, 2:7); 
    
    % Call the wrapper function
    visualize_stanford_arm(q_current);
    
    % Small pause to control animation speed
    %pause(0.001); 
end

disp('Simulation complete.');



function visualize_stanford_arm(q)
    % Accepts q = [j1, j2, d3, j4, j5, j6]
    
    % Extract joint variables
    j1 = q(1); 
    j2 = q(2); 
    d3 = q(3);
    j4 = q(4); 
    j5 = q(5); 
    j6 = q(6);
    
    % System Parameters
    L2 = 2;
    d3 = max(2, min(10, d3)); % clamp d3 limit 2-10
    
    % --- Local Transformation Matrices ---
    T01 = [cos(j1), -sin(j1), 0, 0;
           sin(j1),  cos(j1), 0, 0;
           0,        0,       1, 0;
           0,        0,       0, 1];
           
    T12 = [cos(j2), -sin(j2), 0, 0;
           0,        0,       1, L2;
          -sin(j2), -cos(j2), 0, 0;
           0,        0,       0, 1];
           
    T23 = [1, 0,  0, 0;
           0, 0, -1, -d3;
           0, 1,  0, 0;
           0, 0,  0, 1];
           
    T34 = [cos(j4), -sin(j4), 0, 0;
           0,        0,       1, 0;
          -sin(j4), -cos(j4), 0, 0;
           0,        0,       0, 1];
           
    T45 = [cos(j5), -sin(j5), 0, 0;
           0,        0,       1, 0;
          -sin(j5), -cos(j5), 0, 0;
           0,        0,       0, 1];
           
    T56 = [cos(j6), -sin(j6), 0, 0;
           0,        0,       1, 0;
          -sin(j6), -cos(j6), 0, 0;
           0,        0,       0, 1];

    % --- Global Transformations ---
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;
    T05 = T04 * T45;
    T06 = T05 * T56;
    
    % --- Extract Key Points ---
    P0 = [0; 0; 0];
    P2 = T02(1:3, 4);
    P3 = T03(1:3, 4); 
    P6 = T06(1:3, 4); % Wrist location (same physical spot as P3, P4, P5)
    
    % --- Calculate Gripper Geometry ---
    % Using the Z and Y axes of Frame 6 to orient the gripper prongs
    Z6 = T06(1:3, 3); 
    Y6 = T06(1:3, 2);
    
    P_base_end    = P6 + Z6 * 1.5;           % Stem length
    P_left_start  = P_base_end + Y6 * 0.8;   % Left prong base
    P_right_start = P_base_end - Y6 * 0.8;   % Right prong base
    P_left_end    = P_left_start + Z6 * 1.5; % Left prong tip
    P_right_end   = P_right_start + Z6 * 1.5;% Right prong tip

    % --- Plotting ---
    cla; % Clear the current axis for the new frame
    hold on; grid on;
    
    % 1. Base Pedestal (Red)
    % Extending downwards to simulate the visual pedestal from your image
    plot3([0, 0], [0, 0], [-10, P0(3)], 'r-', 'LineWidth', 4);
    plot3(P0(1), P0(2), P0(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
    
    % 2. Shoulder Offset Link (Blue)
    plot3([P0(1), P2(1)], [P0(2), P2(2)], [P0(3), P2(3)], 'b-', 'LineWidth', 5);
    plot3(P2(1), P2(2), P2(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % 3. Prismatic Boom (Green)
    % Calculate a point extending backwards from Joint 2 to create the "boom" tail
    boom_dir = (P3 - P2) / norm(P3 - P2);
    if isnan(boom_dir(1)); boom_dir = [0;0;1]; end % Safety fallback 
    P_tail = P2 - boom_dir * 6; % Tail extends 6 units back
    plot3([P_tail(1), P3(1)], [P_tail(2), P3(2)], [P_tail(3), P3(3)], 'g-', 'LineWidth', 3);
    
    % 4. End Effector / Gripper (Black)
    % Stem
    plot3([P6(1), P_base_end(1)], [P6(2), P_base_end(2)], [P6(3), P_base_end(3)], 'k-', 'LineWidth', 3);
    % Crossbar
    plot3([P_left_start(1), P_right_start(1)], [P_left_start(2), P_right_start(2)], [P_left_start(3), P_right_start(3)], 'k-', 'LineWidth', 3);
    % Prongs
    plot3([P_left_start(1), P_left_end(1)], [P_left_start(2), P_left_end(2)], [P_left_start(3), P_left_end(3)], 'k-', 'LineWidth', 3);
    plot3([P_right_start(1), P_right_end(1)], [P_right_start(2), P_right_end(2)], [P_right_start(3), P_right_end(3)], 'k-', 'LineWidth', 3);
    
    % Format the figure window
    axis([-15 15 -15 15 -15 15]);
    view([45, 30]); % Match the isometric angle from the screenshot
    xlabel('X-axis'); ylabel('Y-axis'); zlabel('Z-axis');
    drawnow;
end