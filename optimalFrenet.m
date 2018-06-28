close all
clear all
clc
set(0, 'DefaultLineLineWidth', 1);
disp('Optimal Frenet Path Planning')

% way points
WPx = [0.0, 10.0, 20.5, 35.0, 70.5];
WPy = [0.0, -6.0, 5.0, 6.5, 0.0];

%% Define obstacles in objects lists
% objx = objects(:,1);
% objy = objects(:,2);
objects = [20.0, 10.0;
    30.0, 6.0;
    30.0, 8.0;
    35.0, 8.0;
    50.0, 3.0];

%% Create a reference path
ds = 0.1;
GenerateTargetCourse = @(wx, wy) CalcSplineCourse(wx, wy, ds);
[RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath] = ...
    GenerateTargetCourse(WPx, WPy);

% Initial state
s0_d = 10.0 / 3.6;  % Current speed [m/s]
d0 = 2.0;  % Current lateral position [m]
d0_d = 0.0;  % Current lateral speed [m/s]
d0_dd = 0.0;  % Current latral acceleration [m/s]
s0 = 0.0;  % Current course position

area = 20.0;  % animation area length [m]

oFrenetPlanner = cOptimalFrenetPlanner();

show_animation = 1;
if show_animation
    figure
end
%% Start simulation
T = 500;
for t = 1:T
    t;
    trajectory = oFrenetPlanner.FrenetOptimalPlanning( ... 
        referencePath, s0, s0_d, d0, d0_d, d0_dd, objects);
    
    % store the second state of the planned trajectory as
    % initial state for the next iteration for the new trajectory
    s0 = trajectory.s(2);
    s0_d = trajectory.ds(2);
    d0 = trajectory.d(2);
    d0_d = trajectory.dd(2);
    d0_dd = trajectory.ddd(2);
    
    
    %     if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
    %         print("Goal")
    %         break
    %     end
    
    if show_animation
        cla;
        plot(RefX, RefY);
        hold on;
        axis equal
        plot(objects(:, 1), objects(:, 2), 'xk');
        plot(trajectory.x(1:end), trajectory.y(1:end), '-ob');
        plot(trajectory.x(1), trajectory.y(1), 'vc');
        %xlim([trajectory.x(1) - area, trajectory.x(1) + area]);
        %ylim([trajectory.y(1) - area, trajectory.y(1) + area]);
        title(['v[km/h]:', num2str(s0_d * 3.6)]);
        grid on;
        drawnow
    end
end

disp('Finish')
if show_animation
    grid(True)
    pause(0.0001)
end