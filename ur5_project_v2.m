function ur5_project_v2()

    % Robot Setup
    a = [0, -0.42500, -0.39225, 0, 0, 0]';
    d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]';
    alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]';

    % Initial joint angles
    qInitial = [0, 0, 0, 0, 0, 0];
    % Inverse Kinematics: set the desired position and orientation. Use Euler angles in radians for orientation
    desiredPos = [0.7, 0.2, 0.2];
    eulAngles = [0, pi/2, 0];
    ik_result = inverse_kinematics_ur5(d, a, alpha, desiredPos, eulAngles, qInitial);
    disp('Model output:');
    disp(' ');
    disp('IK: Joint angles are:');
    disp(ik_result);
    % Forward Kinematics: now, we will compute the forward kinematics using the joint angles from IK
    fk_result = forward_kinematics_ur5(ik_result, d, a, alpha);
    disp('FK: Position and orientation description of end effector (0T6):');
    disp(fk_result);
    disp('_________________________________________________');
    %%
    % Ask the user for input
    disp(' ');
    userResponse = input('Do you want to preview an animation of the Inverse Kinematics? (Y/N): ', 's');

    % Conditional statement based on the user's response
    if strcmpi(userResponse, 'Y')
        % Execute codeA
        disp('Executing animation preview...');
        % Define the robot using Robotics Toolbox
        L = [Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1)), ...
             Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2)), ...
             Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3)), ...
             Revolute('d', d(4), 'a', a(4), 'alpha', alpha(4)), ...
             Revolute('d', d(5), 'a', a(5), 'alpha', alpha(5)), ...
             Revolute('d', d(6), 'a', a(6), 'alpha', alpha(6))];

        ur5 = SerialLink(L, 'name', 'UR5');

        %% Visualization Setup
        % --- MODIFIED LINE ---
        % Define desired position and size in pixels [left, bottom, width, height]
        desiredPosition = [100, 100, 1000, 750]; % Example: 1000px wide, 750px tall
        hFig = figure('Name', 'UR5 Animation', 'Units', 'pixels', 'Position', desiredPosition);
        % --- END MODIFIED LINE ---

        hold on;
        grid on;
        view([-37.5 30]);
        axis([-1 1 -1 1 -0.5 1.5]);
        % axis equal; % IMPORTANT: Uncomment this line if the robot looks stretched/squashed after resizing
        xlabel('x'); ylabel('y'); zlabel('z');

        %% Inverse Kinematics Visualization: Trajectory Parameters
        timesteps = 50;
        px_traj = linspace(0, desiredPos(1), timesteps);
        py_traj = linspace(0, desiredPos(2), timesteps);
        pz_traj = linspace(0, desiredPos(3), timesteps);
        eul = eulAngles;

        %Initial joint configuration
        qPrevious = zeros(1, 6);

        % Use plot function on the current axes (within the resized figure)
        ur5.plot(qInitial, 'workspace', [-2 2 -2 2 -2 2]); % Plots in the current figure

        for t = 1:timesteps
            pos = [px_traj(t), py_traj(t), pz_traj(t)];

            try
                %Compute inverse kinematics
                qNew = inverse_kinematics_ur5(d, a, alpha, pos, eulAngles, qPrevious);
            catch ME
                % warning('Inverse kinematics failed at timestep %d: %s', t, ME.message);
                continue; % Skip this timestep if IK fails
            end

            % Check if qNew is valid (e.g., not NaN) before animating
            if all(isfinite(qNew))
                % Update the robot's configuration in the current figure
                ur5.animate(qNew');
                plot3(pos(1), pos(2), pos(3), 'ro'); % Plot target point
                qPrevious = qNew; % Update previous joint configuration for next IK guess
                drawnow; % Ensure the plot updates visually
                pause(0.1);
            else
                 % warning('Skipping animation for timestep %d due to invalid joint angles (NaN/Inf)', t);
            end
        end
        disp('Trajectory completed.');

    elseif strcmpi(userResponse, 'N')
        % Code for 'N' response (currently empty)
        disp('Animation skipped.');
    else
        disp('Invalid input. Please respond with Y or N.');
    end
end