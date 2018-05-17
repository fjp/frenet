close all
clear all
clc
set(0, 'DefaultLineLineWidth', 1);
disp('Optimal Frenet Path Planning')

% way points
wx = [0.0, 10.0, 20.5, 35.0, 70.5];
wy = [0.0, -6.0, 5.0, 6.5, 0.0];

% obstacle lists
ob = [20.0, 10.0;
      30.0, 6.0;
      30.0, 8.0;
      35.0, 8.0;
      50.0, 3.0];

 ds = 0.1;
GenerateTargetCourse = @(wx, wy) CalcSplineCourse(wx, wy, ds);
[tx, ty, tyaw, tc, runningLength, referencePath] = GenerateTargetCourse(wx, wy);

% Initial state
c_speed = 10.0 / 3.6;  % Current speed [m/s]
c_d = 2.0;  % Current lateral position [m]
c_d_d = 0.0;  % Current lateral speed [m/s]
c_d_dd = 0.0;  % Current latral acceleration [m/s]
s0 = 0.0;  % Current course position

area = 20.0;  % animation area length [m]

oFrenetPlanner = cOptimalFrenetPlanner();

show_animation = 1;
if show_animation
    figure
end
for i = 1:500
    i
    trajectory = oFrenetPlanner.FrenetOptimalPlanning(referencePath, s0, c_speed, c_d, c_d_d, c_d_dd, ob);
    
    s0 = trajectory.s(2);
    c_d = trajectory.d(2);
    c_d_d = trajectory.dd(2);
    c_d_dd = trajectory.ddd(2);
    c_speed = trajectory.ds(2);
    
%     if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
%         print("Goal")
%         break
%     end
        
    if show_animation
            cla;
            plot(tx, ty);
            hold on;
            axis equal
            plot(ob(:, 1), ob(:, 2), 'xk');
            plot(trajectory.x(1:end), trajectory.y(1:end), '-or');
            plot(trajectory.x(1), trajectory.y(1), 'vc');
            xlim([trajectory.x(1) - area, trajectory.x(1) + area]);
            ylim([trajectory.y(1) - area, trajectory.y(1) + area]);
            title(['v[km/h]:', num2str(c_speed * 3.6)]);
            grid on;
            pause(0.0001);
    end
end
            
dips('Finish')
if show_animation
    grid(True)
    pause(0.0001)
end